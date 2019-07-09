// g++ -framework OpenGL -lglfw -O2 main.cpp
#include <iostream>
#include <fstream>
#include <time.h>
#include <Eigen/Dense>

#include <stdio.h>
#include <GLFW/glfw3.h>

#include "ray.h"
#include "camera.h"
#include "triangle.h"
#include "../obj/OBJ_Loader.h"

using namespace std;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

void Loading_obj (vector<Triangle>& T, objl::Loader Loader, string Filename) {
    Loader.LoadFile(Filename);
    for (int i = 0; i < Loader.LoadedMeshes.size(); i++) {
        objl::Mesh curMesh = Loader.LoadedMeshes[i];
        for (int j=0;j<curMesh.Indices.size();j+=3) {
            int i_A = curMesh.Indices[j];
            int i_B = curMesh.Indices[j+1];
            int i_C = curMesh.Indices[j+2];
            Vector4d A; A << curMesh.Vertices[i_A].Position.X, curMesh.Vertices[i_A].Position.Y, curMesh.Vertices[i_A].Position.Z, 1.0;
            Vector4d B; B << curMesh.Vertices[i_B].Position.X, curMesh.Vertices[i_B].Position.Y, curMesh.Vertices[i_B].Position.Z, 1.0;
            Vector4d C; C << curMesh.Vertices[i_C].Position.X, curMesh.Vertices[i_C].Position.Y, curMesh.Vertices[i_C].Position.Z, 1.0;
            Vector3d N_A; N_A << curMesh.Vertices[i_A].Normal.X, curMesh.Vertices[i_A].Normal.Y, curMesh.Vertices[i_A].Normal.Z;
            Vector3d N_B; N_B << curMesh.Vertices[i_B].Normal.X, curMesh.Vertices[i_B].Normal.Y, curMesh.Vertices[i_B].Normal.Z;
            Vector3d N_C; N_C << curMesh.Vertices[i_C].Normal.X, curMesh.Vertices[i_C].Normal.Y, curMesh.Vertices[i_C].Normal.Z;
            Triangle T_temp(A, B, C, N_A, N_B, N_C);
            T.push_back(T_temp);
        }
    }
}

double scroll_def;
void mouseScrollCB(GLFWwindow *window, double x, double y) {
    printf("mouseScrollCB %.1lf %.1lf\n", x, y);
    scroll_def = y;
}


int main() {
    GLFWwindow* window;
    // ライブラリglfw の初期化
    if (!glfwInit()) return -1;
    // ウィンドウを作成
    window = glfwCreateWindow(640, 640, "Hello World", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    // コールバック関数の設定
    {
        // マウス操作時に呼び出す関数を設定
        glfwSetScrollCallback(window, mouseScrollCB);
    }
    // 作成したウィンドウを，OpenGLの描画関数のターゲットにする
    glfwMakeContextCurrent(window);
    
    
    // モデル読み込み https://github.com/Bly7/OBJ-Loader
    vector<Triangle> T_init{};
    // objファイル
    objl::Loader Loader;
    Loading_obj(T_init, Loader, "../obj/teapot.obj");
    
    // カメラインスタンス生成
    Camera cam; // 視点、画面位置を固定
    
    // 平行光源
    Vector3d light_init;
    light_init << -1.0, 1.0, 0.0;
    light_init = light_init/light_init.norm(); // 規格化してlight*N<=1にしないとRGB値が256.0超えて変になるよ
    
    // 描画のループ
    while (!glfwWindowShouldClose(window)) {
        //clock_t start = clock();
        
        // 画面を塗りつぶす
        glClear(GL_COLOR_BUFFER_BIT);
        
        // World座標変換 T_initを変更
        double theta = 0.05;
        for (int i=0;i<T_init.size();i++) {
            double tempx = T_init[i].A(0);
            double tempy = T_init[i].A(2);
            T_init[i].A(0) = cos(theta)*tempx - sin(theta)*tempy;
            T_init[i].A(2) = sin(theta)*tempx + cos(theta)*tempy;
            tempx = T_init[i].B(0);
            tempy = T_init[i].B(2);
            T_init[i].B(0) = cos(theta)*tempx - sin(theta)*tempy;
            T_init[i].B(2) = sin(theta)*tempx + cos(theta)*tempy;
            tempx = T_init[i].C(0);
            tempy = T_init[i].C(2);
            T_init[i].C(0) = cos(theta)*tempx - sin(theta)*tempy;
            T_init[i].C(2) = sin(theta)*tempx + cos(theta)*tempy;
            // 法線
            tempx = T_init[i].N_A(0);
            tempy = T_init[i].N_A(2);
            T_init[i].N_A(0) = cos(theta)*tempx - sin(theta)*tempy;
            T_init[i].N_A(2) = sin(theta)*tempx + cos(theta)*tempy;
            tempx = T_init[i].N_B(0);
            tempy = T_init[i].N_B(2);
            T_init[i].N_B(0) = cos(theta)*tempx - sin(theta)*tempy;
            T_init[i].N_B(2) = sin(theta)*tempx + cos(theta)*tempy;
            tempx = T_init[i].N_C(0);
            tempy = T_init[i].N_C(2);
            T_init[i].N_C(0) = cos(theta)*tempx - sin(theta)*tempy;
            T_init[i].N_C(2) = sin(theta)*tempx + cos(theta)*tempy;
        }
        
        // T_init -> T copy
        vector<Triangle> T{};
        for (int i=0;i<T_init.size();i++) {
            Triangle T_temp(T_init[i].A, T_init[i].B, T_init[i].C, T_init[i].N_A, T_init[i].N_B, T_init[i].N_C);
            T.push_back(T_temp);
        }
        
        // View変換
        Matrix4d view_matrix, temp_matrix1, temp_matrix2;
        temp_matrix1 <<
        1.0, 0.0, 0.0, -cam.origin(0),
        0.0, 1.0, 0.0, -cam.origin(1),
        0.0, 0.0, 1.0, -cam.origin(2),
        0.0, 0.0, 0.0, 1.0;
        temp_matrix2 <<
        cam.x_vec(0), cam.x_vec(1), cam.x_vec(2), 0.0,
        cam.y_vec(0), cam.y_vec(1), cam.y_vec(2), 0.0,
        cam.z_vec(0), cam.z_vec(1), cam.z_vec(2), 0.0,
        0.0, 0.0, 0.0, 1.0;
        view_matrix = temp_matrix2*temp_matrix1;
        
        for (int i=0;i<T.size();i++) {
            T[i].Init(view_matrix*T[i].A, view_matrix*T[i].B, view_matrix*T[i].C);
        }
        
        // 法線と光源も変換
        Matrix3d N_matrix;
        Vector3d light;
        N_matrix <<
        cam.x_vec(0), cam.x_vec(1), cam.x_vec(2),
        cam.y_vec(0), cam.y_vec(1), cam.y_vec(2),
        cam.z_vec(0), cam.z_vec(1), cam.z_vec(2);
        
        for (int i=0;i<T.size();i++) {
            T[i].Init_N(N_matrix*T[i].N_A, N_matrix*T[i].N_B, N_matrix*T[i].N_C);
        }
        light = N_matrix*light_init;
        
        // 投影変換
        Matrix4d  proj_matrix;
        proj_matrix <<
            2*cam.n/cam.w, 0.0, 0.0, 0.0,
            0.0, 2*cam.n/cam.h, 0.0, 0.0,
            0.0, 0.0, -(cam.f+cam.n)/(cam.f-cam.n), -2*cam.f*cam.n/(cam.f-cam.n),
            0.0, 0.0, -1.0, 0.0;
        
        for (int i=0;i<T.size();i++) {
            T[i].Init(proj_matrix*T[i].A, proj_matrix*T[i].B, proj_matrix*T[i].C);
        }
        
        // -> デバイス座標系
        for (int i=0;i<T.size();i++) {
            // 補間量をomegaで割っておく
            T[i].N_A /= T[i].A(3);
            T[i].N_B /= T[i].B(3);
            T[i].N_C /= T[i].C(3);
            
            T[i].A = T[i].A/T[i].A(3);
            T[i].B = T[i].B/T[i].B(3);
            T[i].C = T[i].C/T[i].C(3);
        }
        
        // ビューポート変換 (ラスタライズ)
        float nx = 512;
        float ny = 512;
        float pixel_array[512][512][4] = {}; // (nx,ny,RGB色,z値)
        for (int i=0;i<512;i++) {
            for (int j=0;j<512;j++) {
                pixel_array[i][j][3] = 2.0;
            }
        }
        for (int t=0;t<T.size();t++) {
            T[t].Bounding_box(); // Bounding_box計算
        }
        clock_t start = clock();
        for (int i=0;i<nx;i++) {
            for (int j=0;j<ny;j++) {
                float x_coord = 1.0/nx + i*(2.0/nx) - 1.0;
                float y_coord = 1.0/ny + j*(2.0/ny) - 1.0;
                for (int t=0;t<T.size();t++) {
                    if (T[t].min_x <= x_coord and x_coord <= T[t].max_x) { // BB内ならラスタライズ
                        if (T[t].min_y <= y_coord and y_coord <= T[t].max_y) {
                            // Edge関数の計算
                            float edge_A, edge_B, edge_C;
                            edge_A = (T[t].C(0)-T[t].B(0))*(y_coord-T[t].B(1)) - (T[t].C(1)-T[t].B(1))*(x_coord-T[t].B(0)); // BC×BP
                            edge_B = (T[t].A(0)-T[t].C(0))*(y_coord-T[t].C(1)) - (T[t].A(1)-T[t].C(1))*(x_coord-T[t].C(0)); // CA×CP
                            edge_C = (T[t].B(0)-T[t].A(0))*(y_coord-T[t].A(1)) - (T[t].B(1)-T[t].A(1))*(x_coord-T[t].A(0)); // AB×AP
                            if (edge_A>=0 and edge_B>=0 and edge_C>=0) { // 三角形の内側(表側)であればラスタライズ
                                float lambda_A, lambda_B, lambda_C;
                                float temp = edge_A + edge_B + edge_C;
                                lambda_A = edge_A/temp;
                                lambda_B = edge_B/temp;
                                lambda_C = edge_C/temp;
                                float depth = lambda_A*T[t].A(2) + lambda_B*T[t].B(2) + lambda_C*T[t].C(2); // depth
                                if (depth < pixel_array[i][j][3]) { // depth値が小さい時、
                                    pixel_array[i][j][3] = depth;
                                    for (int temp=0;temp<3;temp++) {
                                        Vector3d N_temp;
                                        N_temp = lambda_A*T[t].N_A + lambda_B*T[t].N_B + lambda_C*T[t].N_C; // 法線ベクトルの補間
                                        N_temp /= N_temp.norm(); // 正規化
                                        pixel_array[i][j][temp] = 0.2 + 0.8*max(0.0, light.dot(N_temp)); // 色の計算
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // ファイル出力 https://programming.pc-note.net/cpp/filestream.html
//        ofstream ofs("image.ppm");
//        ofs << "P3\n" << nx << " " << ny << "\n255\n";
//        // 描画用
//        for (int j=ny-1;j>=0;j--) {
//            for (int i=0;i<nx;i++) {
//                //   .ppmは左上から0,1,2...で描画するよ
//                //       ------------------------
//                //       |0|1|2|...             |
//                //       | |                    |
//                //       | |                    |
//                //       | |                    |
//                //     ^ | |                    |
//                //   j | ------------------------
//                //       ->
//                //       i
//
//                float r = pixel_array[i][j][0];
//                float g = pixel_array[i][j][1];
//                float b = pixel_array[i][j][2];
//                //            float r = pixel_array[j][i];;
//                //            float g = pixel_array[j][i];;
//                //            float b = pixel_array[j][i];;
//                int ir = int(255.99*r); // 0~255の整数値(8bit)でppmに書き込み
//                int ig = int(255.99*g);
//                int ib = int(255.99*b);
//                ofs << ir << " " << ig << " " << ib << endl;
//            }
//        }
        
        // 白色三角形の描画
        {
            glColor3d(1.0, 1.0, 1.0);
            
            for (int i=0;i<512;i++) {
                for (int j=0;j<512;j++) {
                    float color_temp = 1.0*pixel_array[i][j][2];
                    glColor3d(color_temp, color_temp, color_temp);
                    glBegin(GL_POLYGON);
                    glVertex2d(0.0*1.0/nx + i*(2.0/nx) - 1.0, 0.0*1.0/ny + j*(2.0/ny) - 1.0);
                    glVertex2d(2.0*1.0/nx + i*(2.0/nx) - 1.0, 0.0*1.0/ny + j*(2.0/ny) - 1.0);
                    glVertex2d(2.0*1.0/nx + i*(2.0/nx) - 1.0, 2.0*1.0/ny + j*(2.0/ny) - 1.0);
                    glVertex2d(0.0*1.0/nx + i*(2.0/nx) - 1.0, 2.0*1.0/ny + j*(2.0/ny) - 1.0);
                    glEnd();
                }
            }
        }
        
        clock_t end = clock();
        cout << "Time = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
        // 上記描画した図形を表画面のバッファにスワップする
        glfwSwapBuffers(window);
        
        // イベント処理前にパラメータ初期化
        scroll_def = 0.0;
        // 受け取ったイベント（キーボードやマウス入力）を処理する
        glfwPollEvents();
        
        // カメラ位置変更
        cam.n += scroll_def*0.1;
    }
    
    glfwTerminate();
    
    return 0;
}

