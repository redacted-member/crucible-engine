## What and why

## Checklist
- [ ] `ctest` passes; bit-exact hashes and golden renders unchanged (or the change is recorded in `docs/SOUND_CHANGES.md`)
- [ ] No allocation, lock or blocking call on the audio thread
- [ ] New source files carry the SPDX header (`tools/check-spdx.sh`)
- [ ] Formatted with clang-format
- [ ] Commits are signed off (`git commit -s`, Developer Certificate of Origin)
- [ ] I did not read or copy GPL source code for this change (clean-room rule)
