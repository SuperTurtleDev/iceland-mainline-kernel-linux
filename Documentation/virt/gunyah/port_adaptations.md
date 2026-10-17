# Gunyah driver: mainline 7.2 port adaptations

Source: Android common kernel, tag `android17-6.18-2026-06_r1`
(30c0eaeef78d5c19f687ad8418c27b76353ab261). Driver logic is unchanged; every
edit below is an API-form or build-system adaptation.

| # | File | Upstream form | Mainline 7.2 form | Why |
|---|------|---------------|-------------------|-----|
| 1 | `drivers/virt/gunyah/rsc_mgr.h` | `#include <linux/android_kabi.h>` + `ANDROID_KABI_ENUMERATOR_IGNORE()` | header included only when present (`__has_include`), macro otherwise defined as a no-op `static_assert(1)` | Android ABI-tool annotation; no mainline counterpart |
| 2 | `drivers/virt/gunyah/gunyah_vcpu.c`, `vm_mgr.c` | `#include <trace/hooks/gunyah.h>` (restricted vendor hooks `trace_android_rvh_gh_*`) | `#include "gunyah_vendor_hooks.h"`: forwards to the Android header under `CONFIG_ANDROID_VENDOR_HOOKS`, empty inline stubs otherwise | Vendor-hook infrastructure does not exist on mainline; call sites stay identical |
| 3 | `drivers/virt/gunyah/vm_mgr.h` | open-coded loops over `set_direct_map_{invalid,default}_noflush()` | `gunyah_folio_set_direct_map(folio, valid)`, compiled out under `MODULE` | Not exported to modules on mainline arm64. Lent memory stays protected by the hypervisor stage-2 unmap; only the host-side linear-map hardening is lost when built as a module |
| 4 | `drivers/virt/gunyah/rsc_mgr.c` | `of_node_to_fwnode()` | `of_fwnode_handle()` | Helper renamed upstream |
| 5 | `drivers/virt/Kconfig`, `drivers/virt/Makefile`, `arch/arm64/Kbuild` | same three lines in the Android tree | appended unchanged | Only integration points touched in pre-existing files |

## Verification on OnePlus Pad 4 (SM8850, `7.2.0-sm8850`)

Modules: all ten load; `gunyah: Running under Gunyah hypervisor 51/v1`,
`/dev/gunyah` appears, the `gh_rm_core` auxiliary device registers and the
Qualcomm platform hooks enable. `RM rejected message 00000005. Error: -1` at
load is the optional RM log query (unimplemented by this firmware), harmless.
Boot-time loading on the device is `/etc/modules-load.d/gunyah.conf` listing
`gunyah_rsc_mgr gunyah_vcpu gunyah_irqfd gunyah_ioeventfd gunyah_qcom
gunyah_qtvm` (dependencies come from depmod).

`tools/virt/gunyah/gh_vmtest.py` (python3, no dependencies) creates a 1-vCPU
VM from /dev/gunyah with a hand-built DTB and a 16-byte guest payload;
`gh_trace.sh "<mode args>"...` wraps a run in kprobe events on the
demand-paging path.

| Mode | Result |
|------|--------|
| `off lend`  | VM_START in ~2 ms, guest PSCI SYSTEM_OFF -> `GUNYAH_VCPU_EXIT_STATUS` EXITED, 5/5 runs |
| `mmio lend` | guest store to unmapped GPA -> `GUNYAH_VCPU_EXIT_MMIO` write len 4 data 0x1234, then EXITED |
| `off` (unprotected image) | RM rejects VM_CONFIG_IMAGE with MAP_FAILED (9); see limitations |
| `shared lend [none/vmmio/shm]` | lent image + a second SHARE binding the guest writes to: never reaches the host, guest takes a data abort; see limitations |

### Guest DTB requirements learned from the resource manager

The RM validates the VMM-supplied DTB at VM_INIT and answers
`VALIDATE_FAILED (8)` without further detail. Requirements that mattered:

- `/gunyah-vm-config` with `image-name`, `os-type = "linux"`,
  `vm-type = "aarch64-guest"`, `memory { base-address; size-max }`,
  `interrupts { config = "/intc" }`, `vcpus { affinity = "proxy" }`,
  `vdevices { generate = "/hypervisor" }`.
- The RAM node must be exactly `/memory` (`device_type = "memory"`). A
  unit address such as `memory@80000000` is rejected.
- `/cpus` with `#address-cells = 1`, `#size-cells = 0`; one `cpu@N` with
  `device_type = "cpu"` and `reg` per vCPU.
- A GICv3 node (`arm,gic-v3`, `#interrupt-cells = 3`, `interrupt-controller`,
  `reg` = GICD, GICR) referenced by `interrupts/config`.
- `arm,armv8-timer` PPIs with a CPU mask matching the vCPU count
  (`0x108` for one vCPU); `0xf08` is not accepted.

### Limitations on this firmware

- Only protected VMs start: the image memparcel must be lent
  (`GH_VM_ANDROID_LEND_USER_MEM`). With `GH_VM_SET_USER_MEM_REGION` the RM
  answers `MAP_FAILED` at VM_CONFIG_IMAGE and then refuses to deallocate the
  VMID (error 20), so a VMID is lost per attempt.
- A protected VM cannot fall back to "lent image, rest of RAM shared" as
  transparent guest RAM. Two layers refuse it:
  1. The driver registers only lent bindings for demand paging in a
     protected VM (`gunyah_setup_demand_paging`), so a guest access to a
     shared binding is an unregistered RAM access and the hypervisor
     injects a data abort into the guest (observed: the next exit is the
     guest's exception-vector fetch at 0x200). Declaring the range as
     `vmmio-ranges` or as a `vdevices/shm-0` node changes nothing.
  2. With the shared bindings registered as an experiment, the RM accepts
     VM_INIT and the fault reaches the host, SCM assign and the memextent
     donate to the guest's protected-shared extent (label 3) succeed, but
     `addrspace_map` with the VMMIO flag returns Gunyah error 30 DENIED:
     the hypervisor does not let the host map shared memory into a
     protected VM.
  Shared memory for a protected guest is therefore GuestAccept only: the
  host shares an RM memparcel and the guest accepts it by handle into its
  own stage-2. This tree provides that on the VM fd, see "Runtime protected
  share" below.
- The RM log (`/sys/hypervisor/rm_log`) is unimplemented (RM error -1).
- Never fork() a VMM process that holds lent guest RAM. The lent pages are
  pinned (FOLL_LONGTERM), so dup_mmap() copies them eagerly instead of
  marking them COW; the host read of a page it no longer owns at stage-2
  raises a synchronous external abort in copy_page(), the task dies while
  holding mmap_lock and the box hangs (RCU stall in do_exit ->
  acct_collect). Observed once on the Pad 4. Multi-process VMMs must pass
  guest memory as shared memfd/dma-buf mappings, as crosvm does.
- QTVM (authenticated) launch needs a signed image and a PAS ID; untested.

## Runtime protected share (GuestAccept)

Mainline addition, not in the Android source: `drivers/virt/gunyah/vm_mgr_pshare.c`.
It is the in-tree form of DroidVM's out-of-tree `gunyah_share_mod` (which had
to add its own `/dev/gunyah_share` and resolve RM helpers via kallsyms because
the GKI driver could not be patched). Here the ioctls sit on the VM fd itself
and call `gunyah_rm_mem_share()` / `gunyah_rm_mem_reclaim()` directly.

| ioctl (VM fd) | Direction | Effect |
|---|---|---|
| `GH_VM_PROTECTED_SHARE` | `struct gunyah_protected_share`, in/out | Pin a page-aligned userspace buffer (`FOLL_LONGTERM`), SHARE it as one RM memparcel (ACL: guest with the requested RWX subset, host RWX), return `mem_handle`. Physically contiguous runs are merged into one mem entry. |
| `GH_VM_PROTECTED_SHARE_DMABUF` | `struct gunyah_protected_share_dmabuf`, in/out | Same for a page-aligned slice of a DMA-BUF: the attachment pins the exporter's backing, the parcel is sliced from its sg-table. For VM_PFNMAP exporters GUP cannot pin. |
| `GH_VM_PROTECTED_UNSHARE` | `struct gunyah_protected_unshare`, in | Reclaim the parcel by label and unpin. The RM refuses (`MEM_INUSE`, surfaced as `-EBUSY`) while the guest still holds its acceptance; the share then stays pinned. Pages go back to the allocator only after a successful reclaim. |
| `GH_VM_CHECK_EXTENSION` with `GH_CAP_PROTECTED_SHARE` | | Returns 1 when the three ioctls exist. |

Rules the caller must follow:

- Only the VM owner task (the mm that created the VM) may share or unshare,
  like the other memory ioctls.
- The VM must have been started (RM state LOAD/READY/RUNNING/PAUSED); the
  guest VMID is part of the ACL. Before `GUNYAH_VM_START` the call fails with
  `-ENXIO`.
- Labels are unique per VM among protected shares (`-EEXIST`). Towards the RM
  the label carries bit 31 so it cannot collide with a userspace-region label.
- `guest_phys_addr` is informational: the guest picks its IPA when it accepts.
- Nothing is mapped until the guest accepts. The guest side is the DroidVM
  guest additions' `gunyah_guest.ko`: RM `MEM_ACCEPT` by handle with
  `MAP_IPA_CONTIGUOUS`, one sgl `{gpa, memory_size}`, `validate_label = 0`;
  the parcel's total size equals `memory_size`, which that accept form
  requires.
- Order on teardown: guest `MEM_RELEASE`, then host `GH_VM_PROTECTED_UNSHARE`.
  The host never decides on its own that the guest is done: the RM tracks the
  acceptance, and `-EBUSY` from UNSHARE means "guest has not released, retry
  later". (`gunyah_rm_error_remap()` maps `MEM_INUSE` to `-EBUSY` for this;
  the Android source mapped it to `-EINVAL`.)
- `tools/virt/gunyah/gh_sharetest.py` exercises the host side against the
  PSCI_OFF stub guest (no accept): validation, handles, labels, locked_vm
  accounting, the udmabuf path and reclaim at fd close.
- `tools/virt/gunyah/gh_accepttest.py` is the end-to-end test with a guest
  that really accepts. The guest is `gh_rmproxy.S` (244 bytes, bare metal,
  MMU off): a command loop driven through MMIO exits (dump/load memory, msgq
  send/recv, power off). The host dumps the RM-overlaid DTB to find the RM
  msgq capability ids, frames MEM_ACCEPT / MEM_RELEASE itself and verifies:
  accept at a guest-chosen IPA, host->guest and guest->host visibility
  through the buffer, UNSHARE while accepted -> `-EBUSY` with pages still
  pinned, UNSHARE after MEM_RELEASE -> success, the same sequence for a
  udmabuf-backed `GH_VM_PROTECTED_SHARE_DMABUF` (whole buffer and a sliced
  re-share at an offset), and three still-accepted shares reclaimed at fd
  close with locked_vm back to zero. Verified on the Pad 4.
  Build the stub with the command in its header; the test loads
  `gh_rmproxy.bin` from its own directory.

### Guest DTB requirement for a self-accepting guest

The RM only gives a VM an RM-RPC message queue pair when the VM config asks
for one. Without it the overlay adds just `/hypervisor/qcom,gunyah-vm`, and the
guest has no channel for MEM_ACCEPT. `gh_vmtest.build_dtb(rm_rpc=True)` adds:

```
vdevices {
	generate = "/hypervisor";
	rm-rpc {
		vdevice-type = "rm-rpc";
		generate = "/hypervisor/qcom,resource-mgr";
		console-dev;
		message-size = <0xf0>;
		queue-depth = <8>;
		qcom,label = <1>;
	};
};
```

The RM appends `gunyah-resource-manager`, `qcom,resource-manager` and
`qcom,resource-manager-<api>` to the generated node's compatible list itself
(`parse_rm_rpc()` in the RM's `vm_config_parser.c`), and sets
`reg = <tx_capid rx_capid>` as two 64-bit cells. That is the node both the
in-tree `gunyah_rsc_mgr` (as a guest) and the DroidVM guest additions look for.

Two guest-side details that cost debugging time: with the MMU off, bracket
shared-buffer accesses with `dc civac` (clean before loads so the host's dirty
lines reach memory; clean+invalidate around stores so no dirty host line is
written back over them), and remember that a guest resumes from an MMIO read
exit only on the next `GUNYAH_VCPU_RUN`, so the host must round-trip once more
before it may expect the guest's subsequent stores to have happened.
  Shares still live when the VM fd is closed are reclaimed after the RM has
  reset the VM (the reset drops the guest's acceptances). A parcel the RM
  still refuses to return is leaked, never freed: the hypervisor may still map it.

The RM misc device gets a 64-bit DMA mask at probe so it can serve as the
DMA-BUF importer device.
