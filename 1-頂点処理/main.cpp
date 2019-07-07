#include <iostream>
#include <fstream>
#include <time.h>
#include <Eigen/Dense>

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

int main() {
    //clock_t start = clock();
    
    int nx = 512;
    int ny = 512;
    float pixel_array[512][512] = {};
    
    // モデル読み込み https://github.com/Bly7/OBJ-Loader
    vector<Triangle> T{};
    objl::Loader Loader;
    Loading_obj(T, Loader, "../obj/bunny.obj");
    
    // カメラインスタンス生成
    Camera cam; // 視点、画面位置を固定
    cout << cam.x_vec << endl;
    
    clock_t start = clock();
    
    // World座標変換
    
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
        T[i].A = T[i].A/T[i].A(3);
        T[i].B = T[i].B/T[i].B(3);
        T[i].C = T[i].C/T[i].C(3);
    }
    
    // ビューポート変換
    for (int i=0;i<T.size();i++) {
        int temp_nx_pixel = floor(T[i].A(0)*nx/2.0);
        int temp_ny_pixel = floor(T[i].A(1)*nx/2.0);
        if (fabs(temp_nx_pixel)<=nx/2.0 and fabs(temp_ny_pixel)<=ny/2.0) {
            pixel_array[ny/2+temp_ny_pixel][nx/2+temp_nx_pixel] = 1.0;
        }
        
        temp_nx_pixel = floor(T[i].B(0)*nx/2.0);
        temp_ny_pixel = floor(T[i].B(1)*nx/2.0);
        if (fabs(temp_nx_pixel)<=nx/2.0 and fabs(temp_ny_pixel)<=ny/2.0) {
            pixel_array[ny/2+temp_ny_pixel][nx/2+temp_nx_pixel] = 1.0;
        }
        
        temp_nx_pixel = floor(T[i].C(0)*nx/2.0);
        temp_ny_pixel = floor(T[i].C(1)*nx/2.0);
        if (fabs(temp_nx_pixel)<=nx/2.0 and fabs(temp_ny_pixel)<=ny/2.0) {
            pixel_array[ny/2+temp_ny_pixel][nx/2+temp_nx_pixel] = 1.0;
        }
    }
    
    clock_t end = clock();
    cout << "Time = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
    
    // ファイル出力 https://programming.pc-note.net/cpp/filestream.html
    ofstream ofs("image.ppm");
    ofs << "P3\n" << nx << " " << ny << "\n255\n";
    // 描画用
    for (int j=ny-1;j>=0;j--) {
        for (int i=0;i<nx;i++) {
            float temp_value = pixel_array[j][i];
            
            float r = temp_value;
            float g = temp_value;
            float b = temp_value;
            int ir = int(255.99*r); // 0~255の整数値(8bit)でppmに書き込み
            int ig = int(255.99*g);
            int ib = int(255.99*b);
            ofs << ir << " " << ig << " " << ib << endl;
        }
    }
    
    return 0;
}

