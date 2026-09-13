# StarryOS `PR_GET_AUXV` compatibility

## Problem and scope

Modern libc-independent libraries query the current process auxiliary vector
with `prctl(PR_GET_AUXV)`. The Ubuntu 6.18.3 userspace on the K3 COM260 board
uses this interface during process startup. StarryOS previously returned
`EINVAL`; its `/proc/self/auxv` fallback also omitted the stack-generated
`AT_RANDOM` and `AT_EXECFN` entries. Programs could therefore receive null
pointers and fault before reaching `main`.

The direct users are Linux binaries executed by StarryOS. Success means that a
raw `SYS_prctl` caller observes Linux-compatible copy, return-value, and errno
behavior, and receives the same complete vector that was placed on its initial
stack. The change does not add other `prctl` options, a 32-bit compat ABI, or a
new auxiliary-vector representation.

## Normative behavior

The implementation targets the Linux 6.18 ABI used by the board rootfs:

- [`PR_GET_AUXV(2)`](https://man7.org/linux/man-pages/man2/PR_GET_AUXV.2const.html)
  defines the destination and length arguments, truncation, and the full-size
  return value.
- Linux commit
  [`636e348353a7cc52609fdba5ff3270065da140d5`](https://github.com/torvalds/linux/blob/636e348353a7cc52609fdba5ff3270065da140d5/kernel/sys.c#L2398-L2405)
  copies `min(sizeof(saved_auxv), len)`, skips pointer access for a zero-length
  copy, returns `EFAULT` for a failed non-empty copy, and returns the fixed
  `saved_auxv` size on success.
- The same fixed commit checks the reserved fourth and fifth arguments before
  attempting the copy
  ([`kernel/sys.c`](https://github.com/torvalds/linux/blob/636e348353a7cc52609fdba5ff3270065da140d5/kernel/sys.c#L2692-L2696)),
  so nonzero reserved arguments take `EINVAL` precedence over a bad pointer.
- Linux 6.18 reserves 22 generic auxiliary-vector entries
  ([`include/linux/auxvec.h`](https://github.com/torvalds/linux/blob/v6.18/include/linux/auxvec.h#L6-L8)).
  StarryOS adds the Linux 6.18 architecture-specific entry count and the
  terminating `AT_NULL` slot to reproduce `mm_struct::saved_auxv` size.

`PR_GET_AUXV` is Linux-specific, so no POSIX contract applies.

## Design and alternatives

StarryOS already stores the executable's auxiliary vector in `ProcessData` for
`/proc/[pid]/auxv`. The selected design keeps that object as the single state
source:

1. The ELF loader adds `AT_RANDOM` and `AT_EXECFN` to the saved vector before
   serializing that same vector and an `AT_NULL` terminator onto the initial
   user stack.
2. `PR_GET_AUXV` snapshots the saved vector under its read lock, releases the
   lock, serializes it into the Linux 6.18 fixed-size zero-filled array, and
   copies only the requested prefix to userspace.
3. The fixed array has room for a terminator. Exceeding the architecture's
   Linux capacity is treated as an internal bad state instead of silently
   truncating entries.

Only fixing `/proc/self/auxv` was rejected because constrained processes may
not have procfs and modern callers prefer `PR_GET_AUXV`. Returning the dynamic
StarryOS vector length was rejected because Linux returns the size of its fixed
`saved_auxv` array. Adding a second saved-array field was rejected because it
would duplicate process state and require synchronization across fork and
exec.

## Safety, concurrency, and rollback

All user writes go through `vm_write_slice`, preserving the existing user-memory
validation and `EFAULT` translation. No lock is held across the possibly
faulting write. The change adds no `unsafe`, credential rule, namespace state,
blocking operation, or new public Rust API. Fork continues to clone the same
saved vector, while exec atomically replaces it with the new image's vector.

Rollback consists of removing the `PR_GET_AUXV` match arm and loader update;
there is no persistent data migration. Doing so restores the old `EINVAL`
behavior but makes the K3 Ubuntu userspace unusable under StarryOS.

## Validation

`qemu/system/bugfix-bug-prctl-get-auxv` invokes `SYS_prctl` directly and checks:

- a full copy containing `AT_PAGESZ`, non-null `AT_RANDOM`, non-null
  `AT_EXECFN`, and `AT_NULL`;
- a short exact-prefix copy that still returns the full size;
- a zero-length call with an invalid pointer that succeeds without access;
- `EFAULT` for a non-empty invalid destination;
- `EINVAL` precedence for nonzero reserved arguments.

The same source is run on host Linux as a differential test. The original
StarryOS implementation must fail the QEMU case, the fixed implementation must
pass it, and final K3 validation must reach `root@starry:` with the Ubuntu
rootfs.
