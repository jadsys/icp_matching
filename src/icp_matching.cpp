/**
* @file     icp_matching.cpp
* @brief    ICPによる点群データのマッチングを行うクラスIcpMatchingの実装ソースファイル
* @author   S.Kumada
* @date     2023/09/06
* @details  点群データをマッチングし、補正情報を配信する
*/

#include "icp_matching/icp_matching.h"

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void print4x4Matrix (const Eigen::Matrix3d & mat_rotation, Eigen::Vector3d & vec_translation)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", mat_rotation (0, 0), mat_rotation (0, 1), mat_rotation (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", mat_rotation (1, 0), mat_rotation (1, 1), mat_rotation (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", mat_rotation (2, 0), mat_rotation (2, 1), mat_rotation (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", vec_translation (0), vec_translation (1), vec_translation (2));
}

IcpMatching::IcpMatching(ros::NodeHandle node)
{
    ros::NodeHandle param_node("~"); // パラメータ取得用
    std::string sub_source_cloud_topic;    // 変換元の点群データ受信トピック名
    std::string sub_target_cloud_topic;  // 変換先の点群データ受信トピック名
    std::string pub_correct_info_topic; // 補正値の配信トピック名
    std::string pub_res_cloud_topic;    // 補正後の点群データの配信トピック名
    is_new_target_cloud_ = false; // 点群データの受信フラグ
    is_new_source_cloud_ = false; // 点群データの受信フラグ
        
    state_ = MODE_INITIALIZE; // 内部ステータスの更新

    // パラメータ読み込み
    getParam(param_node, "source_cloud_topic_name",    sub_source_cloud_topic,     std::string("/source_cloud")); // 変換対象の点群データ
    getParam(param_node, "target_cloud_topic_name",  sub_target_cloud_topic,   std::string("/target_cloud")); // 変換先の点群データ
    getParam(param_node, "correction_info_topic_name",  pub_correct_info_topic,  std::string("/correction_info"));  // 求めた補正値の配信
    getParam(param_node, "result_cloud_topic_name",    pub_res_cloud_topic,  std::string("/result_cloud"));    // 変換後の点群データ
    getParam(param_node, "result_cloud_frame_name",     result_cloud_frame_name_,     std::string("icp_result"));    // icp結果の点群データの原点情報
    getParam(param_node, "publish_times",    publish_times_,    0);     // 補正後の点群データの配信回数
    getParam(param_node, "icp_option/maximum_iterations",    maximum_iterations_,    20);   // icpの実行回数   
    getParam(param_node, "icp_option/max_correspondence_distance",    max_correspondence_distance_,    std::sqrt(std::numeric_limits<double>::max())); // 2点間の最大距離の許容値
    getParam(param_node, "icp_option/transformation_epsilon",    transformation_epsilon_,    0.0);  // 1つ前の結果との並行移動成分の差分の許容値
    getParam(param_node, "icp_option/transformation_rotation_epsilon",    transformation_rotation_epsilon_,    0.0);    // 1つ前の結果との回転成分の差分の許容値
    getParam(param_node, "icp_option/euclidean_fitness_epsilon",    euclidean_fitness_epsilon_,    -std::sqrt(std::numeric_limits<double>::max())); // 点群の2乗平均誤差の許容値

    // サブスクライバ定義
    sub_source_cloud_  = node.subscribe(sub_source_cloud_topic, 1, &IcpMatching::recvSourceCloudCB, this);
    sub_target_cloud_ = node.subscribe(sub_target_cloud_topic, 1, &IcpMatching::recvTargetCloudCB, this);
    
    // パブリッシャ定義
    pub_correct_pose_ = node.advertise<uoa_poc6_msgs::r_map_pose_correct_info>(pub_correct_info_topic, 1, true);
    pub_matching_res_cloud_ = node.advertise<pcl::PointCloud<pcl::PointXYZ>>(pub_res_cloud_topic, 1, true);
}

IcpMatching::~IcpMatching()
{
    // 何もしない
}

#if 1
MODE_STATE IcpMatching::getIcpMatchingState(void)
{
    return state_;
}

void IcpMatching::recvSourceCloudCB(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

#ifdef DEBUG
    ROS_INFO("DEBUG : Receive source cloud");
#endif
    sensor_msgs::PointCloud2 cloud_data = *msg; 
    source_frame_ = msg->header.frame_id;   // 補正値のフレーム名
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_data, pcl_cloud); // sensor_msgs::PointCloud2⇒pcl::PointCloud形式への変換
    
    bool equal = false; // 更新フラグ
    
    if( pcl_cloud.points.size() == source_cloud_.points.size())
    {  // データサイズの大きさが同じ場合
        for(int idx = 0; idx < pcl_cloud.points.size() && equal; idx++)
        {
            // 受信データと1つ前の受信データの要素を比較
            equal = fabs(pcl_cloud.points[idx].x - source_cloud_.points[idx].x) <= DBL_EPSILON * fmax(1, fmax(fabs(pcl_cloud.points[idx].x), fabs(source_cloud_.points[idx].x))) &&
                    fabs(pcl_cloud.points[idx].y - source_cloud_.points[idx].y) <= DBL_EPSILON * fmax(1, fmax(fabs(pcl_cloud.points[idx].y), fabs(source_cloud_.points[idx].y))) &&
                    fabs(pcl_cloud.points[idx].z - source_cloud_.points[idx].z) <= DBL_EPSILON * fmax(1, fmax(fabs(pcl_cloud.points[idx].z), fabs(source_cloud_.points[idx].z)));
        }
    }

    if(!equal)
    { // 更新されたcloudデータを受信した場合
        source_cloud_ = pcl_cloud;   // 受信点群の格納
        is_new_source_cloud_ = true;    // 更新された点群データの受信完了

        if((state_ == MODE_INITIALIZE ||    // 初期化済み
            state_ == MODE_FINISHED)  &&    // 補正値配信済み
            is_new_target_cloud_)           // 変換先の点群データが受信完了
        { // ステータスが初期化または、完了状態かつtargetの点群が受信完了の場合
            state_ = MODE_STANDBY;  // スタンバイモードへの移行
        }
        
    }
}

void IcpMatching::recvTargetCloudCB(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

#ifdef DEBUG
    ROS_INFO("DEBUG : Receive target cloud");
#endif
   sensor_msgs::PointCloud2 cloud_data = *msg;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_data, pcl_cloud);
    
    bool equal = false;

    // 受信データの検証
    if( pcl_cloud.points.size() == target_cloud_.points.size())
    {  // データサイズの大きさが同じ場合
        int idx = 0;
        do
        {
            // データの要素を比較（要素が一致の場合：True、不一致の場合：False）
            equal = fabs(pcl_cloud.points[idx].x - target_cloud_.points[idx].x) <= DBL_EPSILON * fmax(1, fmax(fabs(pcl_cloud.points[idx].x), fabs(target_cloud_.points[idx].x))) &&
                    fabs(pcl_cloud.points[idx].y - target_cloud_.points[idx].y) <= DBL_EPSILON * fmax(1, fmax(fabs(pcl_cloud.points[idx].y), fabs(target_cloud_.points[idx].y))) &&
                    fabs(pcl_cloud.points[idx].z - target_cloud_.points[idx].z) <= DBL_EPSILON * fmax(1, fmax(fabs(pcl_cloud.points[idx].z), fabs(target_cloud_.points[idx].z)));
        }
        while((idx < pcl_cloud.points.size()) && equal);    // equalの結果（要素同士が一致の場合true）とインデックスがデータサイズ以下の場合にループ
    }

    // 受信点群の更新判定
    if(!equal)
    { // 更新されたcloudデータを受信した場合
        target_cloud_ = pcl_cloud;   // 受信点群の格納
        is_new_target_cloud_ = true;    // 更新された点群データの受信完了

        // 変換準備完了判定
        if((state_ == MODE_INITIALIZE ||    // 初期化
            state_ == MODE_FINISHED)  &&    // 配信完了
            is_new_source_cloud_)           // 変換元の点群データが受信完了
        { // ステータスが初期化または、完了状態かつsourceの点群が受信完了の場合
            state_ = MODE_STANDBY;  // スタンバイモードへの移行
        }

    }
}

void IcpMatching::executeTransformation()
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    state_ = MODE_PROCESSING;

    // 終了条件設定
    icp.setMaximumIterations(maximum_iterations_);                      // 最大繰り返し回数（デフォルト10回） 
    icp.setTransformationEpsilon(transformation_epsilon_);              // 1つ前の結果との並行移動成分の差分の許容値
    icp.setTransformationRotationEpsilon(transformation_rotation_epsilon_); // 1つ前の結果との回転成分の差分の許容値
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);     // 2点間の最大距離の許容値
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);         // 点群の2乗平均誤差の許容値

#ifdef DEBUG
    // 設定値の確認
    std::cout << "DEBUG : Maximum Iterations : " << icp.getMaximumIterations() << std::endl;
    std::cout << "DEBUG : Max Correspondence Distance : " << icp.getMaxCorrespondenceDistance() << std::endl;
    std::cout << "DEBUG : Transformation Epsilon : " << icp.getTransformationEpsilon() << std::endl;
    std::cout << "DEBUG : Transformation Rotation Epsilon : " << icp.getTransformationRotationEpsilon() << std::endl;
    std::cout << "DEBUG : Euclidean Fitness Epsilon : " << icp.getEuclideanFitnessEpsilon() << std::endl;
    
    // 処理時間計測開始
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
#endif

    // 点群の設定
    icp.setInputSource(source_cloud_.makeShared());    // 変換元
    icp.setInputTarget(target_cloud_.makeShared());  // 変換先

    pcl::PointCloud<pcl::PointXYZ> result_cloud; // icp実行後のクラウドデータ
    icp.align(result_cloud);
    // icp.align(result_cloud, guess); // guessは初期位置(PointCloud2のデータには地図の原点情報は無いため、独自メッセージでの定義が必要)
    
#ifdef DEBUG
    // ICP実行時間
    std::chrono::time_point<std::chrono::system_clock> checkpoint_align;
    checkpoint_align = std::chrono::system_clock::now();
#endif
    //変換matrixを表示する
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    transformation_matrix = icp.getFinalTransformation().cast<double>();

#ifdef DEBUG
    // ICP実行結果取得時間
    std::chrono::time_point<std::chrono::system_clock> checkpoint_get_final_trance;
    checkpoint_get_final_trance = std::chrono::system_clock::now();
#endif

    Eigen::Matrix3d mat; //rotation matrix
    Eigen::Vector3d trans; //translation vector
    
    trans << transformation_matrix(0,3), transformation_matrix(1,3), transformation_matrix(2,3);
    mat   << transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
             transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
             transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2);

#ifdef DEBUG // 確認用
    tf::Matrix3x3 tf_mat;
    tf::matrixEigenToTF(mat, tf_mat);
    double yaw, pitch, roll;
    tf_mat.getEulerYPR(yaw, pitch, roll);

	tf::Quaternion tf_quat;
    ROS_INFO("DEBUG : tf Quaternion");
    tf_mat.getRotation(tf_quat);
    std::cout << "DEBUG : tf Quaternion : " << tf_quat.getX() << ", " << tf_quat.getY() << ", " << tf_quat.getZ() << ", " << tf_quat.getW() << std::endl;
    // std::cout << "tf mat" << tf_mat << std::endl;
    std::cout << "DEBUG : yaw : " << yaw << " [rad]"<< std::endl;
#endif

#ifdef DEBUG
    // 実行速度
    std::chrono::time_point<std::chrono::system_clock> end;
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::chrono::duration<double> elapsed_seconds_align = checkpoint_align- start;
    std::chrono::duration<double> elapsed_seconds_final_trance = checkpoint_get_final_trance - checkpoint_align;

    std::cout<<"DEBUG : checkpoint_align: " << elapsed_seconds_align.count() << "s\n"; // icp実行時間
    std::cout<<"DEBUG : checkpoint_get_final_trance: " << elapsed_seconds_final_trance.count() << "s\n"; // 実行結果取得
    std::cout<<"DEBUG : elapsed time: " << elapsed_seconds.count() << "s\n"; // 総処理時間
    
    // 変換行列
    print4x4Matrix(transformation_matrix);
    Eigen::Vector3d euler = mat.eulerAngles(0,1,2);
    printf ("DEBUG : euler(roll, pitch, yaw ) = < %6.3f, %6.3f, %6.3f >\n\n", euler (0), euler (1), euler (2));
    std::cout << "DEBUG : ICP has converged:" << icp.hasConverged() << "\n" // 収束したか
    << "DEBUG : score: " << icp.getFitnessScore() << "\n"  // 二乗平均の誤差[m]
    << std::endl;
    std::cout << icp.hasConverged() << "," << icp.getFitnessScore() << ","<< elapsed_seconds.count() << ","<< transformation_matrix (0, 3) << ","<<   (1, 3) << ","<<  euler (2) << std::endl;
#endif

    // 補正情報の配信
#ifdef DEBUG
    ROS_INFO("DEBUG : Pub Correct Pose");
#endif
    Eigen::Quaterniond quat(mat); //rotation matrix stored as a quaternion

#ifdef DEBUG // 確認用
    ROS_INFO("DEBUG : Eigen Quaternion");
    std::cout << "Eigen Quaternion : " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << std::endl;
    // std::cout << "Eigen mat" << mat << std::endl;
    std::cout << "yaw : " << euler (2) << " [rad], " << rad_to_deg(euler (2)) << "[deg]" << std::endl;
#endif


    geometry_msgs::Pose curr_pose;
    curr_pose.position.x = trans[0];
    curr_pose.position.y = trans[1];
    curr_pose.position.z = trans[2];
    curr_pose.orientation.x = quat.x();
    curr_pose.orientation.y = quat.y();
    curr_pose.orientation.z = quat.z();
    curr_pose.orientation.w = quat.w();

    uoa_poc6_msgs::r_map_pose_correct_info pub_correct_info;

    pub_correct_info.header.stamp = ros::Time::now();
    pub_correct_info.header.frame_id = source_frame_;
    pub_correct_info.pose = curr_pose;
    pub_correct_info.rmse = icp.getFitnessScore();

    pub_correct_pose_.publish(pub_correct_info); //publishing the source pose

    // マッチング結果のポイントクラウドデータ配信
#ifdef DEBUG
    ROS_INFO("DEBUG : Pub PointCloud Data");
#endif
    auto pub_msg = result_cloud.makeShared(); // スマートポインタ
    pub_msg->header.frame_id = result_cloud_frame_name_;
    ros::Time time_st = ros::Time::now (); 
    pcl_conversions::toPCL(ros::Time::now(), pub_msg->header.stamp); // ROSの配信用msgへ変換 
    
    ros::Rate sleep_rate = ROS_RATE_10HZ; // 配信レート

    state_ = MODE_FINISHED;

    while (ros::ok() &&  state_ == MODE_FINISHED)
    {
        sleep_rate.sleep();
        ros::spinOnce();

        if(publish_times_ <= 0)
        {   // 配信回数分補正結果の点群データを配信する
            break;
        }

        --publish_times_; // 配信カウント-1

        pub_matching_res_cloud_.publish(pub_msg);

    }
}
#endif