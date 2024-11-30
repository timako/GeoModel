#include "../include/GLProgram.h"
#include "glm/ext.hpp"
#include "../include/tinynurbs/tinynurbs.h"
#include "../include/tinynurbs/util/util.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"


// shader source code paths

const char* vertexShaderPath = "../shaders/vertexShader.vs";
const char* whiteFragmentShaderPath = "../shaders/whiteFragmentShader.fs";
const char* redFragmentShaderPath = "../shaders/redFragmentShader.fs";
const char* greenFragmentShaderPath = "../shaders/greenFragmentShader.fs";
const char* blueFragmentShaderPath = "../shaders/blueFragmentShader.fs";

const char* lightvertexShaderPath = "../shaders/lightvertexShader.vs";
const char* fragmentShaderPath = "../shaders/fragmentShader.fs";
//const char* imguiFontPath = "../imgui/misc/fonts/DroidSans.ttf";  //����·��

const char* imguiFontPath = "../imgui/misc/fonts/DroidSans.ttf";  //����·��

GLProgram::GLProgram() :
    deltaTime(0.0f), prevTime(0.0f) {
    surfaceRender.clear();
    contourCurveDisplayer.clear();
    trajectoryCurveDisplayer.clear();
    meshRender.clear();
    normalRender.clear();
}

void DisplayerInit(const tinynurbs::RationalCurve<float>& curve, CurveDisplay& displayer) {
    float u, delta;
    vector<glm::vec3> crvvertex;
    crvvertex.resize(1000);
    u = 0;
    delta = 1.0 / (3000 / 3 - 1);
    // Rendering contour curves
    for (int i = 0; i < 3000 / 3; i++)
    {
        glm::vec3 tmp = tinynurbs::curvePoint(curve, u);

        crvvertex[i] = tmp;
        u += delta;
    }
    displayer.curveRender.Initial(vertexShaderPath, whiteFragmentShaderPath, crvvertex);

    vector<glm::vec3> crvcontrol_points;
    for (int i = 0; i < curve.control_points.size(); i++)
    {
        crvcontrol_points.push_back(curve.control_points[i]);
    }

    displayer.curveControlPointLineRender.Initial(vertexShaderPath, greenFragmentShaderPath, crvcontrol_points);


    float p = 0.5;
    vector<vector<glm::vec3>> Vertices;
    vector<vector<glm::vec3>> OffsetVertex;

    glm::vec3 tmp = tinynurbs::curvePoint(curve, p);

    std::vector<glm::vec3> derivateData = tinynurbs::curveDerivatives(curve, 1, p);

    glm::vec3 tmp1 = derivateData[1];  //����һ��ĵ���������
    double length = sqrt(pow(tmp1.x, 2) + pow(tmp1.y, 2) + pow(tmp1.z, 2)); //������λ����
    tmp1.x = tmp1.x / length;
    tmp1.y = tmp1.y / length;
    tmp1.z = tmp1.z / length;
    glm::vec3 tmp2; //�������������ݵ�
    double len = 1; //���������
    tmp2.x = tmp.x - tmp1.x * len;
    tmp2.y = tmp.y - tmp1.y * len;
    tmp2.z = tmp.z - tmp1.z * len;


    Vertices.resize(1);
    OffsetVertex.resize(1);

    Vertices[0].push_back(tmp);
    OffsetVertex[0].push_back(tmp2);

    displayer.curveDerivateRender.Initial(vertexShaderPath, redFragmentShaderPath, Vertices, OffsetVertex);
}

void GLProgram::DisplayingCurve(const tinynurbs::RationalCurve<float>& curve, CurveDisplay& displayer,
    bool show_curves, bool show_control_points, bool show_derivate, float curves_derivate_u) {
    if (show_curves)
        displayer.curveRender.Draw(camera, modelMatrix, lightPos, windowWidth, windowHeight);
    if (show_control_points)
        displayer.curveControlPointLineRender.Draw(camera, modelMatrix, lightPos, windowWidth, windowHeight);
    if (show_derivate) {
        float p = curves_derivate_u;

        glm::vec3 tmp = tinynurbs::curvePoint(curve, p);

        std::vector<glm::vec3> derivateData = tinynurbs::curveDerivatives(curve, 1, p);

        DisplayingVec({ derivateData[1] }, { tmp }, displayer.curveDerivateRender, 1);
    }
}

void GLProgram::DisplayingVec(const vector<glm::vec3>& Vector, const vector<glm::vec3>& Position,
    NormalRender& Render, double len) {
    vector<vector<glm::vec3>> Vertices;
    vector<vector<glm::vec3>> OffsetVertex;
    Vertices.clear();
    OffsetVertex.clear();
    Vertices.resize(1);
    OffsetVertex.resize(1);
    for (int i = 0; i < Vector.size(); i++) {
        glm::vec3 tmp1 = Vector[i];
        // Normalization
        double length = sqrt(pow(tmp1.x, 2) + pow(tmp1.y, 2) + pow(tmp1.z, 2));
        tmp1.x = tmp1.x / length;
        tmp1.y = tmp1.y / length;
        tmp1.z = tmp1.z / length;
        glm::vec3 tmp2;
        // Position with offset
        tmp2.x = Position[i].x - tmp1.x * len;
        tmp2.y = Position[i].y - tmp1.y * len;
        tmp2.z = Position[i].z - tmp1.z * len;
        OffsetVertex[0].push_back(tmp2);
    }
    Vertices[0].assign(Position.begin(), Position.end());
    Render.Initial(vertexShaderPath, redFragmentShaderPath, Vertices, OffsetVertex);
    Render.Draw(camera, modelMatrix, lightPos, windowWidth, windowHeight);
}


void GLProgram::init(const vector<tinynurbs::RationalCurve<float>>& contour_curves,
    const vector<tinynurbs::RationalCurve<float>>& trajectory_curves,
    const vector<vector<tinynurbs::RationalCurve<float>>>& profile_curves,
    const vector<vector<vector<glm::vec3>>>& frames
    ) {

    vector<tinynurbs::RationalSurface<float>> surfaces;   
    tinynurbs::RationalSurface<float>sweep_surf
    (2,2,{0,0,0,0.15,0.3,0.4,0.6,0.7,0.85,1,1,1,},{0,0,0,0.161836,0.259023,0.36176,0.449662,0.528558,0.600314,0.667673,0.762996,1,1,1,}, 
    //row,col,data of array2
    {9,11,{glm::fvec3(2.28842e-08,-0.5,0.5),glm::fvec3(0.48417,-0.249791,0.780435),glm::fvec3(1.35815,0.153899,0.794649),glm::fvec3(1.74586,0.27785,0.630892),glm::fvec3(2.06772,0.317353,0.462628),glm::fvec3(2.47702,0.240809,0.284369),glm::fvec3(2.73502,0.131627,0.207552),glm::fvec3(3.17556,-0.0852526,0.204113),glm::fvec3(3.67244,-0.366419,0.393393),glm::fvec3(4.52056,-1.00277,0.993507),glm::fvec3(5,-1.5,1.5),glm::fvec3(2.58263e-08,-0.5,1),glm::fvec3(0.512206,-0.210369,1.29785),glm::fvec3(1.58335,0.370408,1.20973),glm::fvec3(1.95331,0.744145,0.786392),glm::fvec3(2.21764,0.78156,0.288975),glm::fvec3(2.67786,0.548481,-0.0795535),glm::fvec3(2.90828,0.191067,-0.269885),glm::fvec3(3.45405,-0.166219,-0.213422),glm::fvec3(4.11539,-0.348706,0.133832),glm::fvec3(5.00754,-0.965124,0.880974),glm::fvec3(5.49053,-1.43151,1.43151),glm::fvec3(2.64208e-08,1,1),glm::fvec3(0.563469,1.3066,1.19878),glm::fvec3(1.60121,1.74901,0.490785),glm::fvec3(1.82629,1.29699,-0.730571),glm::fvec3(2.21758,0.236113,-1.17448),glm::fvec3(2.64893,-0.63112,-1.09388),glm::fvec3(2.87378,-1.32941,-0.478939),glm::fvec3(3.35298,-1.65212,0.0142322),glm::fvec3(4.01114,-1.84473,-0.0781053),glm::fvec3(5.03973,-2.41323,0.49843),glm::fvec3(5.62048,-2.84555,0.948186),glm::fvec3(2.46555e-08,1,0.7),glm::fvec3(0.546648,1.28295,0.88833),glm::fvec3(1.46608,1.6191,0.241734),glm::fvec3(1.70182,1.01722,-0.823871),glm::fvec3(2.12764,-0.0424113,-1.07029),glm::fvec3(2.52842,-0.815723,-0.875531),glm::fvec3(2.76982,-1.36507,-0.192476),glm::fvec3(3.18589,-1.60354,0.264753),glm::fvec3(3.74537,-1.85536,0.0776315),glm::fvec3(4.74755,-2.43582,0.565949),glm::fvec3(5.32616,-2.88664,0.989278),glm::fvec3(2.30823e-08,9.84445e-09,0.5),glm::fvec3(0.501258,0.255866,0.747411),glm::fvec3(1.3641,0.613433,0.555),glm::fvec3(1.70352,0.462133,0.125238),glm::fvec3(2.06771,0.135537,-0.0251922),glm::fvec3(2.46737,-0.152392,-0.0537412),glm::fvec3(2.72352,-0.375199,0.137867),glm::fvec3(3.14187,-0.580552,0.279998),glm::fvec3(3.63769,-0.865095,0.322747),glm::fvec3(4.5313,-1.48547,0.865992),glm::fvec3(5.04332,-1.97135,1.33889),glm::fvec3(2.23018e-08,1,0.3),glm::fvec3(0.524219,1.25141,0.474395),glm::fvec3(1.28592,1.4459,-0.090334),glm::fvec3(1.53586,0.64418,-0.948271),glm::fvec3(2.0077,-0.413777,-0.931371),glm::fvec3(2.36774,-1.06186,-0.584393),glm::fvec3(2.63122,-1.41263,0.189473),glm::fvec3(2.96309,-1.53877,0.598782),glm::fvec3(3.39101,-1.86953,0.285281),glm::fvec3(4.35797,-2.46593,0.655976),glm::fvec3(4.93373,-2.94144,1.04407),glm::fvec3(2.05366e-08,1,-3.24963e-09),glm::fvec3(0.507398,1.22776,0.163945),glm::fvec3(1.1508,1.31599,-0.339385),glm::fvec3(1.41139,0.364403,-1.04157),glm::fvec3(1.91776,-0.692301,-0.82718),glm::fvec3(2.24724,-1.24647,-0.366039),glm::fvec3(2.52726,-1.44829,0.475936),glm::fvec3(2.796,-1.49019,0.849303),glm::fvec3(3.12523,-1.88016,0.441017),glm::fvec3(4.06578,-2.48852,0.723495),glm::fvec3(4.63942,-2.98253,1.08516),glm::fvec3(1.99421e-08,-0.5,7.9355e-09),glm::fvec3(0.456135,-0.289213,0.263017),glm::fvec3(1.13295,-0.0626108,0.379564),glm::fvec3(1.53841,-0.188444,0.475392),glm::fvec3(1.91781,-0.146854,0.63628),glm::fvec3(2.27617,-0.0668641,0.648292),glm::fvec3(2.56176,0.0721867,0.684989),glm::fvec3(2.89707,-0.00428574,0.621649),glm::fvec3(3.22948,-0.384132,0.652954),glm::fvec3(4.03359,-1.04041,1.10604),glm::fvec3(4.50947,-1.56849,1.56849),glm::fvec3(2.28842e-08,-0.5,0.5),glm::fvec3(0.48417,-0.249791,0.780435),glm::fvec3(1.35815,0.153899,0.794649),glm::fvec3(1.74586,0.27785,0.630892),glm::fvec3(2.06772,0.317353,0.462628),glm::fvec3(2.47702,0.240809,0.284369),glm::fvec3(2.73502,0.131627,0.207552),glm::fvec3(3.17556,-0.0852526,0.204113),glm::fvec3(3.67244,-0.366419,0.393393),glm::fvec3(4.52056,-1.00277,0.993507),glm::fvec3(5,-1.5,1.5),}},{9,11,1});
    surfaces.push_back(sweep_surf);
    // lighting
    lightPos = glm::vec3(10.f, 4.0f, 10.0f);
    
    // initialize window system
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    this->window = glfwCreateWindow(this->windowWidth,this-> windowHeight, "3D Surface Plotter", NULL, NULL);
    if (this->window == NULL) {
        std::cout << "FAILED TO CREATE GLFW WINDOW" << std::endl;
        glfwTerminate();
        exit(-1);
    }

    glfwSetWindowUserPointer(window, this);

    glfwMakeContextCurrent(this->window);
    glfwSetFramebufferSizeCallback(this->window, _framebufferSizeCallback);
    glfwSetScrollCallback(window, _scrollCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR);
    glfwSetMouseButtonCallback(window, _mouseButtonCallback);
    glfwSetCursorPosCallback(window, _cursorPosCallback);

    // initialize GLAD before making OpenGL calls
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "FAILED TO INITIALIZE GLAD" << std::endl;
        exit(-1);
    }

    // GL calls
    glViewport(0, 0, this->windowWidth, this->windowHeight);
    glEnable(GL_DEPTH_TEST);

    this->surfaceRender.resize(surfaces.size());

    this->meshRender.resize(surfaceRender.size());

    this->normalRender.resize(surfaceRender.size());

    this->surfaceControlRender.resize(surfaceRender.size());

    this->UsurfaceDerivateRender.resize(surfaceRender.size());
    this->VsurfaceDerivateRender.resize(surfaceRender.size());

    uint degree_contour_curves = contour_curves[0].degree;
    uint degree_trajectory_curves = trajectory_curves[0].degree;
    std::vector<float> knots_contour_curves = contour_curves[0].knots;
    std::vector<float> knots_trajectory_curves = trajectory_curves[0].knots;
    uint num_controlpoints_contour_curves = contour_curves[0].control_points.size();
    uint num_controlpoints_trajectory_curves = trajectory_curves[0].control_points.size();


    for (int k = 0; k < this->surfaceRender.size(); k++)
    {
        /*Original method */
        {
        float u, v, delta;
        u = 0;
        v = 0;
        int numX = 40;
        int numY = 40;
        vector<vector<glm::vec3>> Vertices;
        vector<vector<glm::vec3>> Normal;
        vector<vector<glm::vec3>> OffsetVertex;

        Normal.resize(numX);
        Vertices.resize(numX);
        OffsetVertex.resize(numX);
        delta = 1.0 / (numX - 1);
        for (int x = 0; x < numX; x++) {
            v = 0;
            Vertices[x].resize(numY);
            OffsetVertex[x].resize(numY);
            Normal[x].resize(numY);
            for (int y = 0; y < numY; y++) {
                // add vertex
                // glm::vec3 tmp = tinynurbs::surfacePoint(surfaces[k], u, v);

                glm::vec3 tmp_contour_curve = tinynurbs::curvePoint(contour_curves[k], u);
                glm::vec3 tmp_trajectory_curve = tinynurbs::curvePoint(trajectory_curves[k], v);
                std::vector<glm::vec3> frame_trajectory_curve = tinynurbs::curveTNBFrame<float>(trajectory_curves[k], v);
                std::vector<glm::vec3> frame_trajectory_curve_zero = tinynurbs::curveTNBFrame<float>(trajectory_curves[k], 0);

                
                glm::vec3 T0 = frame_trajectory_curve_zero[0];  
                glm::vec3 N0 = frame_trajectory_curve_zero[1]; 
                glm::vec3 B0 = frame_trajectory_curve_zero[2];  

                glm::vec3 T1 = frame_trajectory_curve[0];  
                glm::vec3 N1 = frame_trajectory_curve[1];  
                glm::vec3 B1 = frame_trajectory_curve[2];  


                glm::mat3x3 frame_zero(T0, N0, B0);  
                glm::mat3x3 frame_v(T1, N1, B1);     

                glm::mat3x3 rotation_matrix = frame_v * transpose(frame_zero);

                glm::mat3x3 scale_matrix = glm::mat3x3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
                glm::mat3x3 rot_scale = rotation_matrix * scale_matrix;

                glm::vec3 tmp =  rot_scale * (tmp_contour_curve) + tmp_trajectory_curve;


                glm::vec3 tmp1 = tinynurbs::surfaceNormal(surfaces[k], u, v);

                double length = sqrt(pow(tmp1.x, 2) + pow(tmp1.y, 2) + pow(tmp1.z, 2)); 

                tmp1.x = tmp1.x / length;
                tmp1.y = tmp1.y / length;
                tmp1.z = tmp1.z / length;
            
                glm::vec3 tmp2; 

                double len = 0.2; 
                tmp2.x = tmp.x - tmp1.x * len;
                tmp2.y = tmp.y - tmp1.y * len;
                tmp2.z = tmp.z - tmp1.z * len;

                Vertices[x][y] = tmp;
                OffsetVertex[x][y] = tmp2;
                Normal[x][y] = tmp1;
                v += delta;
            }
            u += delta;
        }
        this->surfaceRender[k].Initial(lightvertexShaderPath, fragmentShaderPath, Vertices, Normal);

        this->meshRender[k].Initial(vertexShaderPath, whiteFragmentShaderPath, Vertices);
 
        vector<vector<glm::vec3>> MeshVertex;

        MeshVertex.resize(surfaces[k].control_points.rows());

        for (int i = 0; i < surfaces[k].control_points.rows(); i++)
        {
            MeshVertex[i].resize(surfaces[k].control_points.cols());
            for (int j = 0; j < surfaces[k].control_points.cols(); j++) {
                MeshVertex[i][j] = surfaces[k].control_points[surfaces[k].control_points.cols() * i + j];

            }

        }
        this->surfaceControlRender[k].Initial(vertexShaderPath, whiteFragmentShaderPath, MeshVertex);
        }

    }


    this->contourCurveDisplayer.resize(contour_curves.size());
    this->trajectoryCurveDisplayer.resize(trajectory_curves.size());
    this->profileCurveDisplayer.resize(contour_curves.size());
    this->frameDisplayer.resize(contour_curves.size());

    for (int k = 0; k < this->contourCurveDisplayer.size(); k++)
    {
        DisplayerInit(contour_curves[k], contourCurveDisplayer[k]);
        DisplayerInit(trajectory_curves[k], trajectoryCurveDisplayer[k]);
        profileCurveDisplayer[k].resize(profile_curves[k].size());
        frameDisplayer[k].resize(frames[k].size());
        for (int j = 0; j < profile_curves[k].size(); j++) {
            DisplayerInit(profile_curves[k][j], profileCurveDisplayer[k][j]);
            frameDisplayer[k][j].resize(3);
        }
    }

    // initialize IMGUI
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    ImGui::StyleColorsLight();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 450");
    io.Fonts->AddFontFromFileTTF(imguiFontPath, 32.0f);

}

void GLProgram::run(const vector<tinynurbs::RationalCurve<float>>& contour_curves,
    const vector<tinynurbs::RationalCurve<float>>& trajectory_curves,
    const vector<vector<tinynurbs::RationalCurve<float>>>& profile_curves,
    const vector<vector<vector<glm::vec3>>>& frames,
    const vector<vector<float>> v_bar,
    const vector<tinynurbs::RationalSurface<float>>& surfaces) {


    ImGuiIO& io = ImGui::GetIO();
    // imgui state
    bool show_grid = false;
    bool show_tangent = false;

    bool show_normal = false;
    float tangent_u = 0.5f, tangent_v = 0.5f;
    bool show_control_mesh = false;

    bool show_shade_suf = false;


    bool show_contour = false;
    bool show_contour_derivate = false;
    bool show_contour_control_points = false;
    float contour_derivate_u = 0.5f;

    bool show_trajectory = false;
    bool show_trajectory_derivate = false;
    bool show_trajectory_control_points = false;
    float trajectory_derivate_u = 0.5f;

    bool show_frame = false;

    bool show_profile = false;
    bool show_profile_derivate = false;
    bool show_profile_control_points = false;
    float profile_derivate_u = 0.5f;

    bool show_knot_insertion = false;
    bool show_knot_removal = false;
    bool show_degree_elevation = false;
    bool show_degree_reduction = false;
 
    // main loop
    while (!glfwWindowShouldClose(this->window)) {

        // per-frame time logic
        float currTime = glfwGetTime();
        this->deltaTime = currTime - this->prevTime;
        this->prevTime = currTime;


        // input
        enableControl = (!io.WantCaptureMouse && !io.WantCaptureKeyboard);

        // input
        processInput();

        glClearColor(this->clearColor.r, this->clearColor.g, this->clearColor.b, this->clearColor.alpha);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // --- imgui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Settings");
        ImGui::Checkbox("show grid", &show_grid);
        ImGui::Checkbox("show tangent", &show_tangent);
        if (show_tangent || show_normal)
        {
            ImGui::SliderFloat("u##1", &tangent_u, 0.0f, 1.0f);
            ImGui::SliderFloat("v##1", &tangent_v, 0.0f, 1.0f);
        }
        ImGui::Checkbox("show normal", &show_normal);

        ImGui::Checkbox("show shade surface", &show_shade_suf);

        ImGui::Checkbox("show control mesh", &show_control_mesh);

        ImGui::Checkbox("show contour curves", &show_contour);

        ImGui::Checkbox("show contour curves derivate", &show_contour_derivate);

        ImGui::Checkbox("show contour curve control points line", &show_contour_control_points);

        ImGui::Checkbox("show trajectory curves", &show_trajectory);

        ImGui::Checkbox("show trajectory curves derivate", &show_trajectory_derivate);

        ImGui::Checkbox("show trajectory curve control points line", &show_trajectory_control_points);

        ImGui::Checkbox("show frames", &show_frame);

        ImGui::Checkbox("show profile curves", &show_profile);

        ImGui::Checkbox("show profile curves derivate", &show_profile_derivate);

        ImGui::Checkbox("show profile curve control points line", &show_profile_control_points);

        if (show_contour_derivate)
        {
            ImGui::SliderFloat("u##2", &contour_derivate_u, 0.0f, 1.0f);
        }

        if (show_trajectory_derivate)
        {
            ImGui::SliderFloat("u##2", &trajectory_derivate_u, 0.0f, 1.0f);
        }

        if (show_profile_derivate)
        {
            ImGui::SliderFloat("u##2", &profile_derivate_u, 0.0f, 1.0f);
        }

        /*
        ImGui::Checkbox("show control mesh", &show_control_mesh);
        ImGui::Checkbox("show knot insertion", &show_knot_insertion);
        ImGui::Checkbox("show knot removal", &show_knot_removal);
        ImGui::Checkbox("show degree elevation", &show_degree_elevation);
        ImGui::Checkbox("show degree reduction", &show_degree_reduction);
        */

        ImGui::End();
 
        //Draw Surfaces
        for (int k = 0; k < this->surfaceRender.size(); k++)
        {

            vector<vector<glm::vec3>> Vertices;
            vector<vector<glm::vec3>> OffsetVertex;
            Vertices.resize(1);
            OffsetVertex.resize(1);

            Vertices[0].resize(1);
            OffsetVertex[0].resize(1);

            float up, vp;
            up = tangent_u;
            vp = tangent_v;

            glm::vec3 tmp = tinynurbs::surfacePoint(surfaces[k], up, vp);

            if (show_tangent) {
                tinynurbs::array2<glm::vec3> res = tinynurbs::surfaceDerivatives(surfaces[k], 1, up, vp);
                DisplayingVec({ res(1, 0) }, { tmp }, UsurfaceDerivateRender[k], 0.2);
                DisplayingVec({ res(0, 1) }, { tmp }, VsurfaceDerivateRender[k], 0.2);
            }
            if (show_normal) {
                DisplayingVec({ tinynurbs::surfaceNormal(surfaces[k], up, vp) }, {tmp}, normalRender[k], 0.2);
            }

            if (show_shade_suf)
                this->surfaceRender[k].Draw(camera, modelMatrix, lightPos, windowWidth, windowHeight);
        }
        //Draw Curves
        for (int k = 0; k < this->contourCurveDisplayer.size(); k++)
        {
            DisplayingCurve(contour_curves[k], contourCurveDisplayer[k], show_contour, show_contour_control_points,
                show_contour_derivate, contour_derivate_u);
            DisplayingCurve(trajectory_curves[k], trajectoryCurveDisplayer[k], show_trajectory, 
                show_trajectory_control_points, show_trajectory_derivate, trajectory_derivate_u);
            for (int j = 0; j < this->profileCurveDisplayer[k].size(); j++)
            {
                DisplayingCurve(profile_curves[k][j], profileCurveDisplayer[k][j], show_profile,
                    show_profile_control_points, show_profile_derivate, profile_derivate_u);
                if (show_frame) {
                    glm::vec3 tmp = tinynurbs::curvePoint(trajectory_curves[k], v_bar[k][j]);
                    for (int i = 0; i < 3; i++) {
                        DisplayingVec({ frames[k][j][i] }, { tmp }, frameDisplayer[k][j][i], 1);
                    }
                }
            }
        }
        //Draw Mesh
        for (int k = 0; k < this->surfaceRender.size(); k++)
        {
            if (show_grid)
                this->meshRender[k].Draw(camera, modelMatrix, lightPos, windowWidth, windowHeight);
            if(show_control_mesh)
                this->surfaceControlRender[k].Draw(camera, modelMatrix, lightPos, windowWidth, windowHeight);
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // check and call events and swap buffers
        glfwSwapBuffers((this->window));
        glfwPollEvents();
    }
}


void GLProgram::cleanup(void) {

    // clean up gl resources
    for (int k = 0; k < this->surfaceRender.size(); k++)
    {
        glDeleteVertexArrays(1, &(this->surfaceRender[k].VAO));
        glDeleteBuffers(1, &(this->surfaceRender[k].VBO));
        glDeleteBuffers(1, &this->surfaceRender[k].EBO);

        glDeleteVertexArrays(1, &(this->VsurfaceDerivateRender[k].VAO));
        glDeleteBuffers(1, &(this->VsurfaceDerivateRender[k].VBO));
        glDeleteBuffers(1, &this->VsurfaceDerivateRender[k].EBO);

        glDeleteVertexArrays(1, &(this->UsurfaceDerivateRender[k].VAO));
        glDeleteBuffers(1, &(this->UsurfaceDerivateRender[k].VBO));
        glDeleteBuffers(1, &this->UsurfaceDerivateRender[k].EBO);
    }

    for (int k = 0; k < this->contourCurveDisplayer.size(); k++)
    {
        glDeleteVertexArrays(1, &(this->contourCurveDisplayer[k].curveRender.VAO));
        glDeleteBuffers(1, &(this->contourCurveDisplayer[k].curveRender.VBO));
        glDeleteBuffers(1, &this->contourCurveDisplayer[k].curveRender.EBO);

        glDeleteVertexArrays(1, &(this->trajectoryCurveDisplayer[k].curveRender.VAO));
        glDeleteBuffers(1, &(this->trajectoryCurveDisplayer[k].curveRender.VBO));
        glDeleteBuffers(1, &this->trajectoryCurveDisplayer[k].curveRender.EBO);
        for (int j = 0; j < profileCurveDisplayer[k].size(); j++)
        {
            glDeleteVertexArrays(1, &(this->profileCurveDisplayer[k][j].curveRender.VAO));
            glDeleteBuffers(1, &(this->profileCurveDisplayer[k][j].curveRender.VBO));
            glDeleteBuffers(1, &this->profileCurveDisplayer[k][j].curveRender.EBO);
        }
    }

    for (int k = 0; k < this->surfaceRender.size(); k++)
    {
        glDeleteVertexArrays(1, &(this->meshRender[k].VAO));
        glDeleteBuffers(1, &(this->meshRender[k].VBO));
        glDeleteBuffers(1, &this->meshRender[k].EBO);
    }

    for (int k = 0; k < this->normalRender.size(); k++)
    {
        glDeleteVertexArrays(1, &(this->normalRender[k].VAO));
        glDeleteBuffers(1, &(this->normalRender[k].VBO));
        glDeleteBuffers(1, &this->normalRender[k].EBO);
    }
    // clean up glfw
    glfwTerminate();
}

void GLProgram::setClearColor(float r, float g, float b, float alpha) {
    this->clearColor = {r, g, b, alpha};
}

void GLProgram::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    windowWidth = width;
    windowHeight = height;
}

void GLProgram::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {

    if (this->enableControl) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if (action == GLFW_PRESS) {
                glfwGetCursorPos(window, &prevMouseX, &prevMouseY);
                mousePressed = true;
            }
            else if (action == GLFW_RELEASE) {
                mousePressed = false;
            }
        }
    }
}

void GLProgram::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    if(this->enableControl)
        camera.processMouseScroll(yoffset);
}

void GLProgram::cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {

    if (mousePressed && enableControl) {

        // get current cursor coordinates
        double currMouseX, currMouseY;
        glfwGetCursorPos(window, &currMouseX, &currMouseY);

        // get points on arcball
        glm::vec3 va = getArcballVector(prevMouseX, prevMouseY);
        glm::vec3 vb = getArcballVector(currMouseX, currMouseY);

        float speedFactor = 0.1f;
        float angleOfRotation = speedFactor * acos(MIN(1.0f, glm::dot(va, vb)));

        // to get the axis of rotation, need to convert from camera coordinates to world coordinates
        glm::vec3 axisCamera = glm::cross(va, vb);
        glm::mat3 cameraToModel = glm::inverse(glm::mat3(camera.getViewMatrix() * modelMatrix));
        glm::vec3 axisModel = cameraToModel * axisCamera;

        // update model rotation matrix
        float tolerance = 1e-4;
        if (angleOfRotation > tolerance)
            modelMatrix = glm::rotate(modelMatrix, glm::degrees(angleOfRotation), axisModel);

        // update cursor position
        prevMouseX = currMouseX;
        prevMouseY = currMouseY;
    }
}

glm::vec3 GLProgram::getArcballVector(float x, float y) {

    // get normalized vector from center of the virtual arcball to a point P on the arcball's surface
    // if (x,y) is too far away from the arcball, return the nearest point on the arcball's surface
    glm::vec3 P(x/windowWidth * 2 - 1.0, y/windowHeight * 2 - 1.0, 0.0f);
    P.y = -P.y;

    float radius = 1.0f;
    float OP_squared = P.x * P.x + P.y * P.y;

    if (OP_squared <= radius)
        P.z = sqrt(radius - OP_squared); // apply pythagorean theorem to find z
    else
        P = glm::normalize(P); // nearest point

    return P;
}

void GLProgram::processInput(void) {

    // close window with 'ESC' key
    if (glfwGetKey(this->window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(this->window, true);

    // camera movement
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS && enableControl)
        camera.processKeyboard(UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS && enableControl)
        camera.processKeyboard(DOWN, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS && enableControl)
        camera.processKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS && enableControl)
        camera.processKeyboard(RIGHT, deltaTime);
}
