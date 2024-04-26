/**
* @file     icp_matching.hpp
* @brief    ICPによる点群データのマッチングを行うクラスIcpMatchingの定義ヘッダファイル
* @author   S.Kumada
* @date     2023/06/6
* @details  点群データをマッチングし、補正情報を配信するためのクラスの定義ヘッダファイル
*/

#ifndef ICP_MATCHING_H_
#define ICP_MATCHING_H_

#include <iostream>
#include <chrono>

#include <ros/ros.h> // 基本ライブラリ
#include <sensor_msgs/PointCloud2.h> // 配信するデータ型
#include <tf2_ros/transform_listener.h> // 座標変換
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <geometry_msgs/PoseStamped.h>
#include <uoa_poc6_msgs/r_map_pose_correct_info.h>

#if DEBUG   // デバック確認用 
#include <tf/tf.h> // Quaternion変換（デバック用）
#include <tf_conversions/tf_eigen.h>  // Quaternion変換（デバック用）
#endif

#include "utilities.h"

enum MODE_STATE 
{
    MODE_INITIALIZE,    // 初期状態
    MODE_STANDBY,       // 待機中状態
    MODE_PROCESSING,    // ICP実行中状態
    MODE_FINISHED,      // ICP処理完了状態（最終処理実行中）

    MODE_STATE_IDX  // 内部状態数のインデックス
};

/**
 * @brief 点群データのマッチング実行クラス
 * @details 点群データ同士のマッチングを行い、補正値を算出する
 */
class IcpMatching
{
    public:

        /**
        * @brief   IcpMatchingクラスのコンストラクタ
        * @details 初期化を行う
        */
        IcpMatching(ros::NodeHandle node);

        /**
        * @brief   IcpMatchingクラスのデストラクタ
        * @details オブジェクトの破棄等の終了処理を行う
        */
        ~IcpMatching();

        /**
        * @brief        変換元の点群データ受信コールバック関数
        * @param[in]    msg sensor_msgs::PointCloud2::ConstPtr型のメッセージデータ
        * @return       void
        * @details      マッチングを行う、変換元の点群データの受信コールバック関数
        */
        void recvSourceCloudCB(const sensor_msgs::PointCloud2::ConstPtr& msg);

        /**
        * @brief        変換先の点群データ受信コールバック関数
        * @param[in]    msg sensor_msgs::PointCloud2::ConstPtr型のメッセージデータ
        * @return       void
        * @details      マッチングを行う、変換先の点群データの受信コールバック関数
        */
        void recvTargetCloudCB(const sensor_msgs::PointCloud2::ConstPtr& msg);

        /**
        * @brief        ICPマッチングの実行関数
        * @param[in]    void
        * @return       void
        * @details      マッチングを行い、回転行列Rと並進ベクトルT、スコアを取得する
        */
        void executeTransformation();

        /**
        * @brief        マッチング対象の準備ステータスの取得
        * @param[in]    void
        * @return       enum MODE_STATE ステータス値
        * @details      内部ステータスを取得する
        */
        MODE_STATE getIcpMatchingState();

    private:
        ros::Subscriber sub_source_cloud_; //point cloud subscriber
        ros::Subscriber sub_target_cloud_; //point cloud subscriber
        ros::Publisher  pub_correct_pose_; //publishes geometry_msgs::PoseWithCovariance msgs
        ros::Publisher  pub_matching_res_cloud_; //publishes geometry_msgs::PoseWithCovariance msgs
        pcl::PointCloud<pcl::PointXYZ>  source_cloud_;    // 現在点群データ
        pcl::PointCloud<pcl::PointXYZ>  target_cloud_;  // 変換先の点群データ
        std::string source_frame_;  // 受信した変換元のTFフレーム名
        std::string result_cloud_frame_name_;   // 修正後の点群データのTFフレーム名
        int publish_times_; // 補正後の点群データの配信回数
        int maximum_iterations_;    // 最大繰り返し回数（デフォルト10回）
        double transformation_epsilon_; // 1つ前の結果との並行移動成分の差分の許容値
        double transformation_rotation_epsilon_;    // 1つ前の結果との回転成分の差分の許容値
        double max_correspondence_distance_;    // 2点間の最大距離の許容値
        double euclidean_fitness_epsilon_;  // 点群の2乗平均誤差の許容値
        MODE_STATE state_;  // 内部動作モード
        bool is_new_target_cloud_;  // 変換先の点群データの受信フラグ
        bool is_new_source_cloud_;  // 変換元の点群データの受信フラグ

};

#endif