# hsim

The idea with this project is to create a system the can generate sculptures or furniture based on a set of constraints. Specifically, for the first incarnation of this project the constraints were set as follows:

  - The sculpture must be composed of atomic volumes whose 3 dimensions are constrained to ratios from a predefined set. For instance, if we define the ratio set to be [1.0, 0.5] the sculpture can only consist of cubes or rectangles (regardless of absolute size) where one dimension is half of either of the other dimensions of the volume.
  - The sculpture must be stable under the standard laws of physics on earth. This means that if the object was built out of a material of consistent density and all adjoining volumes were fused together, then even with a slight push, it would **not** topple over.
  - The sculpture must be climbable by a cat.

To achieve the above requirements, we implement 3 systems.

  1. A randomized volume generator (custom htree system).
  2. A physics system (Used Nvidia PhysX).
  3. A motion planning system for the cat (parabolic motion planner borrowed from research papers).

We then perform thousands of simulations of randomized volumes until a volume passes all tests -- it stands -- and their is a valid path for the cat. If the tests pass, we consider building the sculpture for our cat.

Simulation screengrabs:
[![Screengrab](https://img.youtube.com/vi/iyXrs_LCj44/0.jpg)](https://www.youtube.com/watch?v=iyXrs_LCj44)

A built sculpture:



# Errata

Got this error on Xcode upgrade:
PhysX/physx/source/foundation/include/unix/PsUnixIntrinsics.h:59:2: error: implicit use of sequentially-consistent atomic may incur stronger memory barriers than necessary [-Werror,-Watomic-implicit-seq-cst]
        __sync_synchronize();
        ^~~~~~~~~~~~~~~~~~
1 error generated.

Had to do this modification
https://github.com/EmbarkStudios/physx-rs/issues/23
...
This can be fixed by modifying the physx-sys/PhysX/physx/source/compiler/cmake/mac/CMakeLists.txt and adding -Wno-atomic-implicit-seq-cst to the SET(PHYSX_CXX_FLAGS line.

