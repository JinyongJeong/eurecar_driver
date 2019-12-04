# lcm_catkin_tester

## How to use lcm headers in catkin_ws
Add set(LCM_DIR ...) and add include_directories in your project CMakeLists.txt

```cmake
set(LCM_DIR "${CATKIN_DEVEL_PREFIX}/include") # <---- add this

include_directories(
  ${LCM_DIR}  # <---- add this
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)
```
