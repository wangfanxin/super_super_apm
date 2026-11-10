#!/usr/bin/env bash
# Read the four unguarded sites the LIO audit identified, in the tree that BUILT the library.
set -u
B=/home/fanxin/lio_avoid/src/Super-LIO/src/super_lio/src
echo "=== ROSWrapper.cpp 440-460 (empty-cloud check) ==="
sed -n '440,460p' $B/ros/ROSWrapper.cpp
echo
echo "=== ROSWrapper.cpp 496-512 (.back() deref) ==="
sed -n '496,512p' $B/ros/ROSWrapper.cpp
echo
echo "=== super_lio.cpp 210-228 (caceData transform) ==="
sed -n '210,228p' $B/lio/super_lio.cpp
echo
echo "=== super_lio_reloc.cpp 306-330 (reloc Output) ==="
sed -n '306,330p' $B/lio/super_lio_reloc.cpp
echo
echo "=== g_pub_step sites ==="
grep -n 'g_pub_step' $B/lio/super_lio.cpp $B/lio/super_lio_reloc.cpp $B/lio/params.cpp $B/ros/ROSWrapper.cpp | head -8
echo
echo "=== effect_mask sizes ==="
grep -n 'effect_mask_\|effect_knn_mask_\|effect_knn_idxs_\|abcd_vec_' $B/../include/lio/super_lio.h 2>/dev/null | head -8
