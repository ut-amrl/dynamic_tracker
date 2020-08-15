set(imguiextern_SOURCE_DIR src)
add_library(imgui SHARED
        ${imguiextern_SOURCE_DIR}/imgui.cpp
        ${imguiextern_SOURCE_DIR}/imgui_demo.cpp
        ${imguiextern_SOURCE_DIR}/imgui_draw.cpp
        ${imguiextern_SOURCE_DIR}/imgui_widgets.cpp
        ${imguiextern_SOURCE_DIR}/examples/imgui_impl_glfw.cpp
        ${imguiextern_SOURCE_DIR}/examples/imgui_impl_opengl3.cpp
        ../gl3w/GL/gl3w.c
)

target_include_directories(imgui PUBLIC
${imguiextern_SOURCE_DIR}
${imguiextern_SOURCE_DIR}/examples
../gl3w
../glfw/include
)

add_library(imgui::imgui ALIAS imgui)
