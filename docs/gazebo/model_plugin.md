# Model Plugin

```cpp title="model_push.hh"
{{include("gz/plugins/hello_model/model_push.hh")}}
```

```cpp title="model_push.cc" linenums="1" hl_lines="13 14 18"
{{include("gz/plugins/hello_model/model_push.cc")}}
```

```cmake title="CMakeLists.txt"
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

SET(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})

add_library(hello_model SHARED hello_model/model_push.cc)
target_link_libraries(hello_model ${GAZEBO_LIBRARIES})
install(TARGETS hello_model DESTINATION bin)
```

```xml title="model_push.world" linenums="1" hl_lines="14 34"
{{include("gz/worlds/model_push.world")}}
```
