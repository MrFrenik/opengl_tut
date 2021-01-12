#include <GLFW/glfw3.h>
#include <gsm/gsm.h>

int32_t main (int32_t argc, char** argv)
{
    assert(glfwInit() != -1);
    
    GLFWwindow* win = glfwCreateWindow(800, 600, "Opengl Tutorial", NULL, NULL);
    assert(win);
    glfwMakeContextCurrent(win);

    // Program loop
    while (!glfwWindowShouldClose(win))
    {
        // Check for input
        if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(win, 1);
        
        // Swap chain and poll key/mouse events
        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}