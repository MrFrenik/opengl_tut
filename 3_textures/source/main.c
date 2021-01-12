#include <GLFW/glfw3.h>
#include <gsm/gsm.h>
#include <common/util.h>

#define ROW_COL_CT  10
#define NUM_CHANNELS 4

const char* vert_src = "\n"
    "#version 410 core\n"
    "layout (location = 0) in vec2 a_pos;\n"
    "layout (location = 1) in vec2 a_uv;\n"
    "out vec2 uv;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = vec4(a_pos, 0.0, 1.0);\n"
    "   uv = a_uv;\n"
    "}\0";

const char* frag_src = "\n"
    "#version 410 core\n"
    "uniform sampler2D u_tex;\n"
    "in vec2 uv;\n"
    "out vec4 frag_color;\n"
    "void main()\n"
    "{\n"
    "   frag_color = texture(u_tex, uv);\n"
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
    glfwSwapInterval(1);
    
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
    // Construct uniform
    int32_t u_tex = glGetUniformLocation(program, "u_tex");

    // Vertex data
    float v_data[] = {
         // Positions   // UV
        -0.5f, -0.5f,   0.0f, 0.0f,
         0.5f, -0.5f,   1.0f, 0.0f,
         0.0f,  0.5f,   0.5f, 1.0f
    };

    uint32_t vbo, vao;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(v_data), v_data, GL_STATIC_DRAW);

    // Position
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // UV
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0); 
    glBindVertexArray(0);

    // Generate procedural texture data (checkered texture)
    uint8_t c0[] = {255, 255, 255, 255};
    uint8_t c1[] = {20, 50, 150, 255};
    uint8_t pixels[ROW_COL_CT * ROW_COL_CT * NUM_CHANNELS] = {0};
    for (uint32_t r = 0; r < ROW_COL_CT; ++r) {
        for (uint32_t c = 0; c < ROW_COL_CT; ++c) {
            const int32_t re = (r % 2) == 0;
            const int32_t ce = (c % 2) == 0;
            uint32_t idx = r * ROW_COL_CT * NUM_CHANNELS + c * NUM_CHANNELS;
            uint8_t* c = (re && ce) ? c0 : (re) ? c1 : (ce) ? c1 : c0;
            pixels[idx + 0] = c[0];
            pixels[idx + 1] = c[1];
            pixels[idx + 2] = c[2];
            pixels[idx + 3] = c[3];
        } 
    }

    // Construct GPU texture
    uint32_t tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); 
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); 
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); 
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, ROW_COL_CT, ROW_COL_CT, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Program loop
    while (!glfwWindowShouldClose(win))
    {
        // Check for input
        if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(win, 1);

        // Render triangle to screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw triangle
        glUseProgram(program);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, tex);
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        // Swap chain and poll key/mouse events
        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}