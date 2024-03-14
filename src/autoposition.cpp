#include "Uwb_Location/autoposition.h"

int aid_last = 0;
int anc_num = 8;
const int nDim = 2;
const int stationNumber = 4;
bool isFirst = true;
int errNum = 0;
const int MAX_ERR_NUM = 3;
double height = 0.55; // 给定z轴坐标

Eigen::MatrixXd matrixBuff = Eigen::MatrixXd::Zero(anc_num, anc_num);
Eigen::MatrixXd ancRangeValuesAvg = Eigen::MatrixXd::Zero(anc_num, anc_num);
Eigen::Matrix<double, stationNumber, stationNumber> g_range_anchor;
Eigen::Matrix<double, stationNumber, stationNumber> g_range_anchor_backup;
Eigen::Matrix<double, stationNumber, stationNumber> g_range_anchor_avg;

bool autopositioning(int *range, int aid, Eigen::MatrixXd& anchorArray) 
{
    //将接收到的基站数据进行备份
    matrixBuff(0, aid) = range[0];
    for (int i = 0; i < anc_num; i++)
    {
        matrixBuff(aid, i) = range[i];
    }
    printf(" aid: %d, aid_last: %d\n",aid, aid_last);

    if(aid != aid_last)
    {
        if(aid <= aid_last)
        {
            //一组数据接收完毕，开始进行基站坐标解算
            std::vector<int> receiveAID;

            //校验buff数据完整性，是否符合已勾选基站数量
            for (int i = 0; i < anc_num; i++)
            {
                for (int j = 0; j < anc_num; j++)
                {
                    if(matrixBuff(i, j) != 0)
                    {
                        //数据有效，无需继续遍历。将当前aid存入receiveAID
                        receiveAID.push_back(i);
                        break;
                    }
                }
            }

            if(receiveAID.size() == stationNumber)   //有效信号数量为设定值
            {
                //将buffer数据填充到矩阵中
                for (int i = 0; i < stationNumber; i++)
                {
                    for (int j = 0; j < stationNumber; j++)
                    {
                        g_range_anchor(i, j) = matrixBuff(receiveAID[i], receiveAID[j]);

                        //对多次矩阵数据做平均值处理
                        if(isFirst)
                        {
                            //首次处理信号，不做平均值处理
                            g_range_anchor_avg(i, j) = g_range_anchor(i, j);
                        }
                        else
                        {
                            //方案1：使用本次信号与上次信号的平均值
                            // g_range_anchor_avg[i][j] = (double)((g_range_anchor[i][j]+g_range_anchor_backup[i][j])/2.0f);
                            //方案2：使用本次信号与上次平均值的平均值
                            g_range_anchor_avg(i, j) = (double)((g_range_anchor(i, j)+ g_range_anchor_avg(i, j))/2.0f);
                        }

                        g_range_anchor_backup(i, j)= g_range_anchor(i, j);
                    }
                }

                // we have 4 ranges (A0 to A1, A0 to A2, A0 to A3, A1 to A2, A1 to A3, A2 to A3)
                // calculate A0, A1, A2 and A3 coordinates (x, y) based on A0 = 0,0 and assume A0-A1 makes the x-axis
                // 同时y轴垂直于x轴，A2, A3的y坐标为正
                int nNodes = stationNumber;
                int viewNode = 0;

                Eigen::MatrixXd  twrdistance = Eigen::MatrixXd::Zero(nNodes, nNodes);
                Eigen::MatrixXd  transCoord = Eigen::MatrixXd::Zero(nNodes, nDim);
                Eigen::MatrixXd  estCoord = Eigen::MatrixXd::Zero(nNodes, nDim);

                for(int i=0;i<stationNumber;i++)
                {
                    for(int j=0;j<stationNumber;j++)
                    {
                        if(i==0)
                            twrdistance(i, j) = g_range_anchor_avg(j, i)/1000;
                        else
                            twrdistance(i, j) = g_range_anchor_avg(i, j)/1000;
                    }
                }

                mds(twrdistance, nNodes, viewNode, transCoord); // Using multi-dimension scaling to estimation the shape

                angleRotation(transCoord, nNodes, estCoord); // estCoord is the coordinates with a1 on the x axis and a2 in the first quadrant

                for(int i=0; i < receiveAID.size(); i++) //  update positions of A0, A1, A2, A3
                {
                    anchorArray(int (receiveAID[i]), 0) = estCoord(i, 0);
                    anchorArray(int (receiveAID[i]), 1) = estCoord(i, 1);
                    anchorArray(int (receiveAID[i]), 2) = height; // 默认所有的基站都处于同一高度
                    //TODO 可以增加传感器，动态调整z轴的坐标
                    //if (i==3) anchorArray(int (receiveAID[i]), 2) = 0.55;
                }

                isFirst = false;
                errNum = 0;
                return true;
            }
            else
            {
                //有效数据缺失，需提醒标定失败
                errNum++;
                if(errNum >= MAX_ERR_NUM)
                {
                    ROS_ERROR_STREAM("Autoposition failed!!!\n");
                    errNum = 0;
                }
            }
            //重置buff数据
            for (int i = 0; i < anc_num; i++)
            {
                for (int j = 0; j < anc_num; j++)
                {
                    matrixBuff(i, j) = 0;
                }
            }
        }
        aid_last = aid;
        return false;
    }
    return false;
}

void mds(Eigen::MatrixXd twrdistance, int nNodes, int viewNode, Eigen::MatrixXd& transCoord)
{
    Eigen::MatrixXd distSquared = Eigen::MatrixXd::Zero(nNodes, nNodes);
    Eigen::MatrixXd u = Eigen::MatrixXd::Zero(nNodes, nNodes);
    Eigen::MatrixXd v = Eigen::MatrixXd::Zero(nNodes, nNodes);
    Eigen::VectorXd s = Eigen::VectorXd::Zero(nNodes);
    Eigen::MatrixXd estGeom = Eigen::MatrixXd::Zero(nNodes, nDim);

    distSquared = twrdistance.array().square();

    Eigen::MatrixXd centeringOperator = Eigen::MatrixXd::Identity(nNodes,nNodes)-(Eigen::MatrixXd::Ones(nNodes,nNodes)/nNodes);

    Eigen::MatrixXd centeredDistSquared = -0.5 * centeringOperator * distSquared * centeringOperator;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(centeredDistSquared, Eigen::ComputeThinU | Eigen::ComputeThinV);
    s = svd.singularValues();
    u = svd.matrixU();
    v = svd.matrixV();

    for (int i = 0; i<nDim; i++)
    {
        estGeom.col(i) = u.col(i)*sqrt(s(i));
    }

    for (int i = 0; i<nNodes; i++)
    {
        transCoord.row(i) = estGeom.row(i)-estGeom.row(viewNode);
    }
}

void angleRotation(Eigen::MatrixXd transCoord, int nNodes, Eigen::MatrixXd& estCoord)
{
    Eigen::MatrixXd rotationMatrix = - Eigen::MatrixXd::Ones(2,2);
    Eigen::VectorXd transdistance = Eigen::VectorXd::Zero(nNodes);
    Eigen::MatrixXd Coord = Eigen::MatrixXd::Zero(nNodes,nDim);

    for (int i = 0; i<nNodes; i++)
    {
        transdistance(i) = sqrt(transCoord(i,0)*transCoord(i,0) + transCoord(i,1)*transCoord(i,1));
    }

    Eigen::VectorXd currentAngle = (transCoord.col(0).array() / transdistance.array()).acos();

    for (int i = 0; i<nNodes; i++)
    {
        if (transCoord(i,1)<0)
        {
            currentAngle(i) = -currentAngle(i);
        }
    }

    double rotateAngle = currentAngle(1);

    rotationMatrix << cos(rotateAngle), -sin(rotateAngle),
                      sin(rotateAngle), cos(rotateAngle);

    Coord = transCoord * rotationMatrix;

    if (Coord(2,1) < 0)
    {
        Coord(2,1) = -Coord(2,1);
    }

    estCoord = Coord;
}

