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

            Triangle T_temp(A, B, C);
            T.push_back(T_temp);
        }
    }
}

void Loading_Floor (vector<Triangle>& T, float floor_x, float floor_y) {
    Vector4d A; A << floor_x,floor_y,floor_x,1.0;
    Vector4d B; B << -floor_x,floor_y,floor_x,1.0;
    Vector4d C; C << floor_x,floor_y,-floor_x,1.0;
    Triangle T_temp(A, B, C);
    T.push_back(T_temp);
    
    A << -floor_x,floor_y,-floor_x,1.0;
    B << -floor_x,floor_y,floor_x,1.0;
    C << floor_x,floor_y,-floor_x,1.0;
    T_temp.Init(A, B, C);
    T.push_back(T_temp);
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
    Loading_obj(T_init, Loader, "../obj/bunny.obj");
    // 床オブジェクト
    //Loading_Floor(T, -15.0, -5.0);
    
    // カメラインスタンス生成
    Camera cam; // 視点、画面位置を固定
    
    // 描画のループ
    while (!glfwWindowShouldClose(window)) {
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
        }
        
        // T_init -> T copy
        vector<Triangle> T{};
        for (int i=0;i<T_init.size();i++) {
            Triangle T_temp(T_init[i].A, T_init[i].B, T_init[i].C);
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
            //cout << T[i].A << endl;
            T[i].Init(view_matrix*T[i].A, view_matrix*T[i].B, view_matrix*T[i].C);
            //cout << T[i].A << endl;
        }
        
        // 投影変換
        Matrix4d  proj_matrix;
        proj_matrix <<
        2*cam.n/cam.w, 0.0, 0.0, 0.0,
        0.0, 2*cam.n/cam.h, 0.0, 0.0,
        0.0, 0.0, -(cam.f+cam.n)/(cam.f-cam.n), -2*cam.f*cam.n/(cam.f-cam.n),
        0.0, 0.0, -1.0, 0.0;
        
        for (int i=0;i<T.size();i++) {
            T[i].Init(proj_matrix*T[i].A, proj_matrix*T[i].B, proj_matrix*T[i].C);
            //cout << T[i].A << endl;
        }
        
        // -> デバイス座標系
        for (int i=0;i<T.size();i++) {
            T[i].A = T[i].A/T[i].A(3);
            T[i].B = T[i].B/T[i].B(3);
            T[i].C = T[i].C/T[i].C(3);
            //cout << T[i].A << endl;
        }
        
        // 白色三角形の描画
        {
            glColor3d(1.0, 1.0, 1.0);
            
            for (int i=0;i<T.size();i++) {
                glBegin(GL_POINTS);
                glVertex2d(T[i].A(0), T[i].A(1));
                glVertex2d(T[i].B(0), T[i].B(1));
                glVertex2d(T[i].C(0), T[i].C(1));
                glEnd();
            }
        }
        
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

