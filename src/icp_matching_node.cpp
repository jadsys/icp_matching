/**
* @file     icp_matching_node.cpp
* @brief    icp_matchingノードのメイン処理ソースファイル
* @author   S.Kumada
* @date     2023/09/05
* @details  点群データをマッチング、補正情報の配信処理を実行する
*/

#include "icp_matching/icp_matching.h"

int main(int argc, char **argv)
{
    // 初期化
    ros::init(argc, argv, "icp_matching");
    ros::NodeHandle node;

    // 地図生成クラスのインスタンス化
    IcpMatching icp_obj(node);

    ros::Rate sleep_rate = ROS_RATE_30HZ; // 30Hz
    // 実行ループ
    while(ros::ok())
    {
        sleep_rate.sleep();
        
        // ステータス監視
        if(icp_obj.getIcpMatchingState() == MODE_STANDBY)
        { // 待機中の場合
            // ICP実行
            icp_obj.executeTransformation();
        }
        
        // コールバック受信待機
        ros::spinOnce();

    }
    

    return (0);
}