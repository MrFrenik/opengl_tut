#ifndef __UTIL_H__
#define __UTIL_H__

#define force_inline static inline

#include <assert.h>
#include <stdio.h>
#include <gl/glad.h>
#include <GLFW/glfw3.h>

// Utility functions for examples
force_inline GLFWwindow* glfw_init_create_window(uint32_t width, uint32_t height, const char* title)
{
    assert(glfwInit() != -1);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    #ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif
    glfwSwapInterval(1);
    GLFWwindow* win = glfwCreateWindow(width, height, title, NULL, NULL);
    assert(win);
    glfwMakeContextCurrent(win);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        printf("Failed to initialize GLFW.\n");
        return NULL;
    }
}

force_inline int32_t util_compile_shader_stage(const char* src, int32_t stage_type)
{
    int32_t ss = glCreateShader(stage_type);
    glShaderSource(ss, 1, &src, NULL);
    glCompileShader(ss);
    // Check for error
    int32_t success; 
    char info_log[512];
    glGetShaderiv(ss, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(ss, 512, NULL, info_log);  
        printf("Error: shader failed to compile: %s", info_log);
    }

    return ss;
}

force_inline int32_t util_create_and_link_shader(int32_t vs, int32_t fs)
{
    int32_t sp = glCreateProgram();
    glAttachShader(sp, vs);
    glAttachShader(sp, fs);
    glLinkProgram(sp);
    // Check for error
    int32_t success; 
    char info_log[512];
    glGetProgramiv(sp, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(sp, 512, NULL, info_log);
        printf("Error: shader failed to link: %s", info_log);
    }
    // Delete shaders (no longer needed now linked)
    glDeleteShader(vs);
    glDeleteShader(fs);
    return sp;
}

#endif // __UTIL_H__ 