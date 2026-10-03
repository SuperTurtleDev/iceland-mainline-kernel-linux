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

Verification on OnePlus Pad 4 (SM8850, `7.2.0-sm8850`): all ten modules load,
`gunyah: Running under Gunyah hypervisor 51/v1`, `/dev/gunyah` appears,
`gh_rm_core` auxiliary device registers, QCOM platform hooks enable.
`RM rejected message 00000005 (-1 UNIMPLEMENTED)` at load is the optional RM
log query and is harmless.
