#ifndef FILEOPT_H
#define FILEOPT_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fstream> 

// using namespace Eigen;
using namespace std;

/**
 * @brief Output trajectory data to a file
 * @brief 输出轨迹数据到文件
 * @param path File path / 文件路径
 * @param data Eigen matrix containing trajectory data / 包含轨迹数据的Eigen矩阵
 * @param row Number of rows to output / 要输出的行数
 * @param col Number of columns to output / 要输出的列数
 */
void traj_out(const char * path, Eigen::MatrixXd & data, int row, int col);

/**
 * @brief Read trajectory data from a text file into a 2D vector
 * @brief 从文本文件读取轨迹数据到二维向量
 * @param path File path / 文件路径
 * @param dof Degrees of freedom (default: 6) / 自由度(默认:6)
 * @return 2D vector containing trajectory data / 包含轨迹数据的二维向量
 */
vector<vector<double>> traj_in(string *path, int dof = 6);

/**
 * @brief Read trajectory data from a text file into vector of Eigen vectors
 * @brief 从文本文件读取轨迹数据到Eigen向量数组
 * @param path File path / 文件路径
 * @param dof Degrees of freedom (default: 6) / 自由度(默认:6)
 * @return Vector of Eigen vectors containing trajectory data / 包含轨迹数据的Eigen向量数组
 */
vector<Eigen::VectorXd> traj_inx(string *path, int dof = 6 );

/**
 * @brief Read trajectory data from a text file into a 2D vector
 * @brief 从文本文件读取轨迹数据到二维向量
 * @param path File path / 文件路径
 * @param dof Degrees of freedom (default: 6) / 自由度(默认:6)
 * @return 2D vector containing trajectory data / 包含轨迹数据的二维向量
 */
vector<vector<double>> traj_in(const char *path, int dof = 6);

/**
 * @brief Read trajectory data from a text file into vector of Eigen vectors
 * @brief 从文本文件读取轨迹数据到Eigen向量数组
 * @param path File path / 文件路径
 * @param dof Degrees of freedom (default: 6) / 自由度(默认:6)
 * @return Vector of Eigen vectors containing trajectory data / 包含轨迹数据的Eigen向量数组
 */
vector<Eigen::VectorXd> traj_inx(const char *path, int dof = 6 );

/**
 * @brief Write trajectory data (vector of Eigen vectors) to a file
 * @brief 将轨迹数据(Eigen向量数组)写入文件
 * @param path File path / 文件路径
 * @param pathpoints Vector of Eigen vectors containing trajectory points / 包含轨迹点的Eigen向量数组
 * @param dof Degrees of freedom (default: 6) / 自由度(默认:6)
 */
void traj_outx(const char*path,vector<Eigen::VectorXd>& pathpoints, int dof = 6);

/**
 * @brief Write trajectory data (2D vector) to a file
 * @brief 将轨迹数据(二维向量)写入文件
 * @param path File path / 文件路径
 * @param temp_vector 2D vector containing trajectory data / 包含轨迹数据的二维向量
 * @param dof Degrees of freedom (default: 6) / 自由度(默认:6)
 */
void traj_out(const char* path,vector<vector<double>> &temp_vector, int dof = 6);

/**
 * @brief Write trajectory data (1D vector) to a file
 * @brief 将轨迹数据(一维向量)写入文件
 * @param path File path / 文件路径
 * @param temp_vector 1D vector containing trajectory data / 包含轨迹数据的一维向量
 */
void traj_out(const char* path,vector<double> &temp_vector );

/**
 * @brief Write trajectory data (vector of Eigen vectors) to a file with scientific notation
 * @brief 将轨迹数据(Eigen向量数组)以科学计数法写入文件
 * @param path File path / 文件路径
 * @param pathpoints Vector of Eigen vectors containing trajectory points / 包含轨迹点的Eigen向量数组
 * @param dof Degrees of freedom (default: 6) / 自由度(默认:6)
 */
void traj_outx_s(const char*path,vector<Eigen::VectorXd>& pathpoints, int dof = 6);

/**
 * @brief Write trajectory data (2D vector) to a file with scientific notation
 * @brief 将轨迹数据(二维向量)以科学计数法写入文件
 * @param path File path / 文件路径
 * @param temp_vector 2D vector containing trajectory data / 包含轨迹数据的二维向量
 * @param dof Degrees of freedom (default: 6) / 自由度(默认:6)
 */
void traj_out_s(const char* path,vector<vector<double>> &temp_vector, int dof = 6);

// void traj_out_s(const char* path,vector<double> &temp_vector );  // Commented out as not implemented


/**
 * @brief Read integer data from a file
 * @brief 从文件读取整数数据
 * @param path File path / 文件路径
 * @param data Pointer to store read data / 存储读取数据的指针
 * @param num Number of integers to read / 要读取的整数数量
 */
void  traj_in(const char *path, int32_t *data, int num);

/**
 * @brief Write integer data to a file
 * @brief 将整数数据写入文件
 * @param path File path / 文件路径
 * @param data Pointer to data to write / 要写入数据的指针
 * @param num Number of integers to write / 要写入的整数数量
 */
void traj_out(const char* path, int32_t *data, int num);

#endif