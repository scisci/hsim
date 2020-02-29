# hsim

Got this error on Xcode upgrade:
PhysX/physx/source/foundation/include/unix/PsUnixIntrinsics.h:59:2: error: implicit use of sequentially-consistent atomic may incur stronger memory barriers than necessary [-Werror,-Watomic-implicit-seq-cst]
        __sync_synchronize();
        ^~~~~~~~~~~~~~~~~~
1 error generated.

Had to do this modification
https://github.com/EmbarkStudios/physx-rs/issues/23
...
This can be fixed by modifying the physx-sys/PhysX/physx/source/compiler/cmake/mac/CMakeLists.txt and adding -Wno-atomic-implicit-seq-cst to the SET(PHYSX_CXX_FLAGS line.