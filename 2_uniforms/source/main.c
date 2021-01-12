#include <GLFW/glfw3.h>
#include <gsm/gsm.h>
#include <common/util.h>

const char* vert_src = "\n"
    "#version 410 core\n"
    "layout (location = 0) in vec2 a_pos;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = vec4(a_pos, 0.0, 1.0);\n"
    "}\0";

const char* frag_src = "\n"
    "#version 410 core\n"
    "uniform vec3 u_color;\n"
    "uniform float u_time;\n"
    "out vec4 frag_color;\n"
    "void main()\n"
    "{\n"
    "   float st = sin(u_time) * 0.5f + 0.5f;\n"
    "   frag_color = vec4(st * u_color, 1.0);\n"
    "}\n\0";

int32_t compile_shader_stage(const char* src, int32_t stage_type)
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

int32_t create_and_link_shader(int32_t vs, int32_t fs)
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

int32_t main (int32_t argc, char** argv)
{
    assert(glfwInit() != -1);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    #ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif
    
    GLFWwindow* win = glfwCreateWindow(800, 600, "Opengl Tutorial", NULL, NULL);
    assert(win);
    glfwMakeContextCurrent(win);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        printf("Failed to initialize GLFW.\n");
        return NULL;
    }

    // Construct Vertex Shader
    int32_t vs = compile_shader_stage(vert_src, GL_VERTEX_SHADER);
    // Construct Fragment Shader
    int32_t fs = compile_shader_stage(frag_src, GL_FRAGMENT_SHADER);
    // Construct Shader Program and Link
    int32_t program = create_and_link_shader(vs, fs);
    // Construct uniforms
    int32_t u_color = glGetUniformLocation(program, "u_color");
    int32_t u_time = glGetUniformLocation(program, "u_time");

    // Vertex data
    float v_data[] = {
         // Positions
        -0.5f, -0.5f,
         0.5f, -0.5f,
         0.0f,  0.5f
    };

    uint32_t vbo, vao;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(v_data), v_data, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0); 
    glBindVertexArray(0);

    // Program loop
    while (!glfwWindowShouldClose(win))
    {
        static float t = 0.f;
        t += 0.0001f;

        // Check for input
        if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(win, 1);

        // Render triangle to screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw triangle
        glUseProgram(program);
        glUniform3f(u_color, 1.0f, 0.1f, 0.3f);
        glUniform1f(u_time, t);
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        // Swap chain and poll key/mouse events
        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}