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
    "out vec4 frag_color;\n"
    "void main()\n"
    "{\n"
    "   frag_color = vec4(u_color, 1.0);\n"
    "}\n\0";

int32_t main (int32_t argc, char** argv)
{
    // Init glfw and construct window
    GLFWwindow* win = glfw_init_create_window(800, 600, "Opengl Tutorial");
    // Construct Vertex Shader
    int32_t vs = util_compile_shader_stage(vert_src, GL_VERTEX_SHADER);
    // Construct Fragment Shader
    int32_t fs = util_compile_shader_stage(frag_src, GL_FRAGMENT_SHADER);
    // Construct Shader Program and Link
    int32_t program = util_create_and_link_shader(vs, fs);
    // Construct uniform
    int32_t u_color = glGetUniformLocation(program, "u_color");

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
        // Check for input
        if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(win, 1);

        // Render triangle to screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw triangle
        glUseProgram(program);
        glUniform3f(u_color, 1.0f, 0.1f, 0.3f);
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        // Swap chain and poll key/mouse events
        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}