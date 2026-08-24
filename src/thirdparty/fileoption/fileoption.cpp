#include "fileoption.h"
// #include "robot.h"

using namespace Eigen;
using namespace std;

/**
 * @brief 将 Eigen 矩阵数据输出到文件
 * @param[in] path 输出文件路径
 * @param[in] data 要输出的矩阵数据
 * @param[in] row 矩阵行数
 * @param[in] col 矩阵列数
 */
void traj_out(const char * path, Eigen::MatrixXd & data, int row, int col)
{
    // 输出操作信息
    cout << "存储文件-->" << path << " . size  : " << row << " * " << col << endl;

    // 创建输出文件流
    ofstream outfile;
    
    // 以截断模式打开文件（覆盖已有内容）
    outfile.open(path, ios::trunc);
    
    // 逐行写入数据
    for (int i = 0; i != row; i++) {
        for (int j = 0; j < col; j++) {
            outfile << data(i, j) << " "; // 写入矩阵元素
        }
        outfile << endl; // 换行
    }
    
    // 关闭文件
    outfile.close();
}

/**
 * @brief 将整型数组数据输出到文件（追加模式）
 * @param[in] path 输出文件路径
 * @param[in] data 要输出的整型数组
 * @param[in] num 数组元素数量
 */
void traj_out(const char* path, int32_t *data, int num)
{
    // 输出操作信息
    cout << "存储文件-->" << path << endl;

    // 以追加模式打开文件
    FILE* fp = fopen(path, "a+");
    
    if (!fp) {
        printf("文件不存在");
    } else {
        // 逐元素写入数据
        for (int i = 0; i < num; i++) {
            int temp = data[i];
            fprintf(fp, "%d ", temp); // 写入整型数据
        }
        fprintf(fp, "\n"); // 换行
        fclose(fp); // 关闭文件
    }
}

/**
 * @brief 从文件读取整型数组数据
 * @param[in] path 输入文件路径
 * @param[out] data 存储读取数据的数组
 * @param[in] num 要读取的元素数量
 */
void traj_in(const char *path, int32_t *data, int num)
{
    // 以只读模式打开文件
    FILE * fp = fopen(path, "r");
    cout << "读取文件-->" << path << endl;

    if (!fp) {
        printf("文件不存在");
    } else {
        // 逐元素读取数据
        for (int i = 0; i < num; ++i) {
            int res = fscanf(fp, "%d 0", &data[i]);
        }
        fclose(fp);
    }
}

/**
 * @brief 将 VectorXd 向量组转换为二维向量并输出到文件
 * @param[in] path 输出文件路径
 * @param[in] pathpoints 包含轨迹点的向量组
 * @param[in] dof 每个点的维度（自由度）
 */
void traj_outx(const char* path, vector<VectorXd>& pathpoints, int dof)
{
    // 创建二维向量容器
    vector<vector<double>> out;
    
    // 将 VectorXd 转换为 vector<double>
    for (int i = 0; i < pathpoints.size(); ++i) {
        vector<double> outline;
        for (int j = 0; j < dof; ++j) 
            outline.push_back(pathpoints[i][j]);
        out.push_back(outline);
    }
    
    // 调用标准输出函数
    traj_out(path, out, dof);
}

/**
 * @brief 将 VectorXd 向量组转换为二维向量并追加到文件
 * @param[in] path 输出文件路径
 * @param[in] pathpoints 包含轨迹点的向量组
 * @param[in] dof 每个点的维度（自由度）
 */
void traj_outx_s(const char* path, vector<VectorXd>& pathpoints, int dof)
{
    // 创建二维向量容器
    vector<vector<double>> out;
    
    // 将 VectorXd 转换为 vector<double>
    for (int i = 0; i < pathpoints.size(); ++i) {
        vector<double> outline;
        for (int j = 0; j < dof; ++j) 
            outline.push_back(pathpoints[i][j]);
        out.push_back(outline);
    }
    
    // 调用追加输出函数
    traj_out_s(path, out, dof);
}

/**
 * @brief 将二维向量数据追加到文件（不覆盖原有内容）
 * @param[in] path 输出文件路径
 * @param[in] out_vector 要输出的二维向量数据
 * @param[in] dof 每行输出的数据点数
 */
void traj_out_s(const char* path, vector<vector<double>> &out_vector, int dof)
{
    // 输出操作信息
    cout << "存储文件-->" << path << " . size : " << out_vector.size();
    if (!out_vector.empty()) 
        cout << " * " << out_vector[0].size();
    cout << endl;

    // 创建输出文件流
    ofstream outfile;
    
    // 以追加模式打开文件
    outfile.open(path, ios::app);
    
    // 逐行写入数据
    for (vector<double>::size_type i = 0; i != out_vector.size(); i++) {
        for (int j = 0; j < dof; j++) {
            outfile << out_vector[i][j] << " ";
        }
        outfile << endl;
    }
    
    // 关闭文件
    outfile.close();
}

/**
 * @brief 将二维向量数据输出到文件（覆盖原有内容）
 * @param[in] path 输出文件路径
 * @param[in] out_vector 要输出的二维向量数据
 * @param[in] dof 每行输出的数据点数
 */
void traj_out(const char* path, vector<vector<double>> &out_vector, int dof)
{
    // 输出操作信息
    cout << "存储文件-->" << path << " . size : " << out_vector.size();
    if (!out_vector.empty()) 
        cout << " * " << out_vector[0].size();
    cout << endl;

    // 创建输出文件流
    ofstream outfile;
    
    // 以截断模式打开文件（覆盖原有内容）
    outfile.open(path, ios::trunc);
    
    // 逐行写入数据，设置高精度输出
    for (vector<double>::size_type i = 0; i != out_vector.size(); i++) {
        for (int j = 0; j < dof; j++) {
            outfile << fixed << setprecision(15) << out_vector[i][j] << " ";
        }
        outfile << endl;
    }
    
    // 关闭文件
    outfile.close();
}

/**
 * @brief 从文件读取数据到 VectorXd 向量组
 * @param[in] path 输入文件路径
 * @param[in] dof 每个数据点的维度
 * @return 包含读取数据的 VectorXd 向量组
 */
vector<VectorXd> traj_inx(string *path, int dof)
{
    // 创建返回容器
    vector<VectorXd> out;
    
    // 读取数据到二维向量
    vector<vector<double>> in = traj_in(path, dof);
    
    // 将二维向量转换为 VectorXd
    for (int i = 0; i < in.size(); ++i) {
        VectorXd aline(dof);
        for (int j = 0; j < dof; ++j) 
            aline[j] = in[i][j];
        out.push_back(aline);
    }
    
    return out;
}

/**
 * @brief 从文件读取数据到二维向量
 * @param[in] path 输入文件路径
 * @param[in] dof 每行数据点的数量
 * @return 包含读取数据的二维向量
 */
vector<vector<double>> traj_in(string *path, int dof)
{
    // 创建文件输入流
    ifstream infile;
    
    // 创建数据容器
    vector<vector<double>> data_vector;
    
    // 打开文件
    infile.open(*path);
    cout << "读取文件-->" << *path << endl;

    // 检查文件是否存在
    if (!infile) {
        cout << "读入文件: " << *path << " 不存在" << endl;
        return data_vector;
    }

    string indata;
    double temp;
    
    // 处理文件内容
    if (infile.is_open()) {
        int line = 1;
        // 逐行读取直到文件结束
        while (infile.good() && !infile.eof()) {
            vector<double> group;
            for (int i = 0; i < dof; i++) {
                if (infile.good() && !infile.eof()) {
                    infile >> indata;
                    istringstream strstream(indata);
                    strstream >> temp;
                } else {
                    break; // 文件结束
                }
                group.push_back(temp);
            }
            line++;
            
            // 检查数据数量是否正确
            if (group.size() == dof) {
                data_vector.push_back(group);
            } else {
                break; // 数据不完整
            }
        }
    }
    
    // 关闭文件
    infile.close();
    cout << "读取完成。size: " << data_vector.size() << " * " << dof << endl;
    
    return data_vector;
}

/**
 * @brief 从文件读取数据到二维向量（char* 路径版本）
 * @param[in] path 输入文件路径
 * @param[in] dof 每行数据点的数量
 * @return 包含读取数据的二维向量
 */
vector<vector<double>> traj_in(const char *path, int dof)
{
    string paths = path;
    return traj_in(&paths, dof);
}

/**
 * @brief 从文件读取数据到 VectorXd 向量组（char* 路径版本）
 * @param[in] path 输入文件路径
 * @param[in] dof 每个数据点的维度
 * @return 包含读取数据的 VectorXd 向量组
 */
vector<Eigen::VectorXd> traj_inx(const char *path, int dof)
{
    string paths = path;
    return traj_inx(&paths, dof);
}

/**
 * @brief 将一维向量数据输出到文件
 * @param[in] path 输出文件路径
 * @param[in] out_vector 要输出的一维向量数据
 */
void traj_out(const char* path, vector<double> &out_vector)
{
    // 输出操作信息
    cout << "存储文件-->" << path << " . size : " << out_vector.size();
    
    // 创建输出文件流
    ofstream outfile;
    
    // 以截断模式打开文件
    outfile.open(path, ios::trunc);
    
    // 逐行写入数据（每行一个元素）
    for (vector<double>::size_type i = 0; i != out_vector.size(); i++) {
        outfile << out_vector[i] << " ";
        outfile << endl;
    }
    
    // 关闭文件
    outfile.close();
}