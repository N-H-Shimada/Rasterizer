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
            Vector3d N_A; N_A << curMesh.Vertices[i_A].Normal.X, curMesh.Vertices[i_A].Normal.Y, curMesh.Vertices[i_A].Normal.Z;
            Vector3d N_B; N_B << curMesh.Vertices[i_B].Normal.X, curMesh.Vertices[i_B].Normal.Y, curMesh.Vertices[i_B].Normal.Z;
            Vector3d N_C; N_C << curMesh.Vertices[i_C].Normal.X, curMesh.Vertices[i_C].Normal.Y, curMesh.Vertices[i_C].Normal.Z;
            Triangle T_temp(A, B, C, N_A, N_B, N_C);
            T.push_back(T_temp);
        }
    }
}

void Loading_Cornel_box (vector<Triangle>& T) {
    Vector4d origin; origin << 0.0,3.0,0.0,1.0;
    float dis = 4.0;
    Vector4d V[8];
    
    for (int i=0;i<8;i++) {
        int temp_i = i;
        int a = temp_i/4;
        temp_i -= a*4;
        int b = temp_i/2;
        temp_i -= b*2;
        int c = temp_i;
        //cout << a << " " << b << " " << c << endl;
        Vector4d temp; temp << a*2.0-1.0, b*2.0-1.0, c*2.0-1.0, 0.0;
        V[i] = temp*4.0 + origin;
        //cout << V[i] << endl;
    }
    int list[8][3] = {
        0,2,1, 1,2,3,
        4,5,6, 5,7,6,
        0,4,2, 2,4,6,
        0,1,4, 1,5,4
    };
    //     2 -----------  6
    //      /|         /|
    //     / |        / |
    //   3 -----------7 |
    //    | 0/-------|--/ 4
    //    | /        | /
    //   1|/_________|/5
    //
    Vector3d N_list[8];
    N_list[0] << 1.0,0.0,0.0; N_list[1] << 1.0,0.0,0.0;
    N_list[2] << -1.0,0.0,0.0; N_list[3] << -1.0,0.0,0.0;
    N_list[4] << 0.0,0.0,1.0; N_list[5] << 0.0,0.0,1.0;
    N_list[6] << 0.0,1.0,0.0; N_list[7] << 0.0,1.0,0.0;
    
    for (int i=0;i<8;i++) {
        Triangle T_temp(V[list[i][0]],V[list[i][1]],V[list[i][2]],N_list[i],N_list[i],N_list[i]);
        T.push_back(T_temp);
    }
}


int main() {
    
    int nx = 512;
    int ny = 512;
    
    // モデル読み込み https://github.com/Bly7/OBJ-Loader
    vector<Triangle> T{};
    objl::Loader Loader;
    Loading_obj(T, Loader, "../obj/teapot.obj");
    //Loading_obj(T, Loader, "../obj/sphere.obj");
    // Cornel_box
    //Loading_Cornel_box(T);
    
    clock_t start = clock();
    
    // カメラインスタンス生成
    Camera cam; // 視点、画面位置を固定
    
    // 平行光源
    Vector3d light;
    light << -1.0, 1.0, 0.0;
    light = light/light.norm(); // 規格化してlight*N<=1にしないとRGB値が256.0超えて変になるよ
    
    // World座標変換
    // ~
    
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
    
    // 法線と光源も変換
    Matrix3d N_matrix;
    N_matrix <<
        cam.x_vec(0), cam.x_vec(1), cam.x_vec(2),
        cam.y_vec(0), cam.y_vec(1), cam.y_vec(2),
        cam.z_vec(0), cam.z_vec(1), cam.z_vec(2);
    
    for (int i=0;i<T.size();i++) {
        T[i].Init(view_matrix*T[i].A, view_matrix*T[i].B, view_matrix*T[i].C);
        T[i].Init_N(N_matrix*T[i].N_A, N_matrix*T[i].N_B, N_matrix*T[i].N_C);
    }
    light = N_matrix*light;
    
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
        for (int j=0;j<3;j++) {
            T[i].A(j) = T[i].A(j)/T[i].A(3);
            T[i].B(j) = T[i].B(j)/T[i].B(3);
            T[i].C(j) = T[i].C(j)/T[i].C(3);
            
            // 補間量をomegaで割っておく
            T[i].N_A(j) /= T[i].A(3);
            T[i].N_B(j) /= T[i].B(3);
            T[i].N_C(j) /= T[i].C(3);
        }
    }
    
    // ビューポート変換 (ラスタライズ)
    float pixel_array[512][512][4] = {}; // (nx,ny,RGB色,z値)
    for (int i=0;i<512;i++) {
        for (int j=0;j<512;j++) {
            pixel_array[i][j][3] = 2.0;
        }
    }
    for (int t=0;t<T.size();t++) {
        T[t].Bounding_box(); // Bounding_box計算
    }
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

//    float pixel_array[512][512] = {};
//    for (int i=0;i<T.size();i++) {
//        int temp_nx_pixel = floor(T[i].A(0)*nx/2.0);
//        int temp_ny_pixel = floor(T[i].A(1)*nx/2.0);
//        if (fabs(temp_nx_pixel)<=nx/2.0 and fabs(temp_ny_pixel)<=ny/2.0) {
//            pixel_array[ny/2+temp_ny_pixel][nx/2+temp_nx_pixel] = 1.0;
//        }
//        //cout << ny/2+temp_ny_pixel << " " << nx/2+temp_nx_pixel << endl;
//
//        temp_nx_pixel = floor(T[i].B(0)*nx/2.0);
//        temp_ny_pixel = floor(T[i].B(1)*nx/2.0);
//        if (fabs(temp_nx_pixel)<=nx/2.0 and fabs(temp_ny_pixel)<=ny/2.0) {
//            pixel_array[ny/2+temp_ny_pixel][nx/2+temp_nx_pixel] = 1.0;
//        }
//        //cout << ny/2+temp_ny_pixel << " " << nx/2+temp_nx_pixel << endl;
//
//        temp_nx_pixel = floor(T[i].C(0)*nx/2.0);
//        temp_ny_pixel = floor(T[i].C(1)*nx/2.0);
//        if (fabs(temp_nx_pixel)<=nx/2.0 and fabs(temp_ny_pixel)<=ny/2.0) {
//            pixel_array[ny/2+temp_ny_pixel][nx/2+temp_nx_pixel] = 1.0;
//        }
//        //cout << ny/2+temp_ny_pixel << " " << nx/2+temp_nx_pixel << endl;
//    }
    
    clock_t end = clock();
    cout << "Time = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
    
    // ファイル出力 https://programming.pc-note.net/cpp/filestream.html
    ofstream ofs("image.ppm");
    ofs << "P3\n" << nx << " " << ny << "\n255\n";
    // 描画用
    for (int j=ny-1;j>=0;j--) {
        for (int i=0;i<nx;i++) {
            //   .ppmは左上から0,1,2...で描画するよ
            //       ------------------------
            //       |0|1|2|...             |
            //       | |                    |
            //       | |                    |
            //       | |                    |
            //     ^ | |                    |
            //   j | ------------------------
            //       ->
            //       i
            
            float r = pixel_array[i][j][0];
            float g = pixel_array[i][j][1];
            float b = pixel_array[i][j][2];
//            float r = pixel_array[j][i];;
//            float g = pixel_array[j][i];;
//            float b = pixel_array[j][i];;
            int ir = int(255.99*r); // 0~255の整数値(8bit)でppmに書き込み
            int ig = int(255.99*g);
            int ib = int(255.99*b);
            ofs << ir << " " << ig << " " << ib << endl;
        }
    }
    
    return 0;
}

