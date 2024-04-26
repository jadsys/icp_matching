/**
* @file     utilities.cpp
* @brief    ユーティリティ実装ソースファイル
* @author   S.Kumada
* @date     2023/09/04
* @details  汎用的な関数の実装ソースコード
*/

#include "utilities.h"

bool checkLastPublishTime(double freq, ros::Time last_pub_time)
{
    bool result = false;
    // 配信間隔時間(nsec)
    uint64_t pub_interval_time_ = freq == 0 ? 0 : 1 / freq * std::pow(10, 9);
    
    uint32_t nsec_part = pub_interval_time_ % 1000000000UL; // 1秒で割った余り
    uint32_t sec_part = pub_interval_time_ / 1000000000UL; // 1秒で割ったときの整数部


    // 時間差
    ros::Duration time_diff = ros::Time::now() - last_pub_time;

    // 周期以上判定
    if(time_diff.sec > sec_part ||  // 周期秒より大きい
        time_diff.sec == sec_part && time_diff.nsec >= nsec_part) // 周期秒と一致かつ周期ナノ秒以上
    {
        result = true;
    }
    
    return result;
}

std::string rosTimeToIso8601(ros::Time time)
{
    struct timespec my_time; // 時刻格納変数(エポック秒)
    char iso_time[40];
    char time_zone[10];
    char converted_time[70];

    my_time.tv_sec = time.sec;
    my_time.tv_nsec = time.nsec;

    strftime(iso_time, sizeof(iso_time)-1,"%FT%T", localtime(&my_time.tv_sec)); // iso時間へ変換
    strftime(time_zone, sizeof(time_zone)-1,"%z", localtime(&my_time.tv_sec)); // タイムゾーン設定
    
    sprintf(converted_time, "%s.%03lu%s", iso_time, (my_time.tv_nsec+500)/1000, time_zone); // 結合

    std::string result_str_time = converted_time;

    return (result_str_time);
}

bool strTimeToRosTime(ros::Time &res_time, std::string date_time, std::string format="%Y-%m-%dT%H:%M:%S%z") 
{
    bool isSuccess = false;
    
    /** test cord **/
    // const char* char_date_time = "2023-01-27T11:13:12.3844+0900";
    // date_time

    time_t time_sec;
    struct tm tm;

    // String⇒時刻
    isSuccess = strptime(date_time.c_str(), format.c_str(), &tm ) != NULL;
    
    // ローカル時間帯⇒協定世界時 (UTC) 
    time_sec = mktime(&tm);
    
    // チェック
    isSuccess = time_sec  =! -1;

    // 秒の格納
    res_time.sec = time_sec;

    // ナノ秒の抽出
    /* 厳密にnsecまで必要な処理が無いため未実装 */
    /* 実装の際は桁数に注意 */

    return isSuccess;
}

std::string iso8601ex(void)
{
    int ch;
    char iso_time[40];
    char time_zone[10];
    char dest[70];
    struct timeval myTime;
    struct tm *time_st;

    memset(iso_time, 0, sizeof(iso_time));
    memset(time_zone, 0, sizeof(time_zone));
    memset(dest, 0, sizeof(dest));

    gettimeofday(&myTime, NULL);
    time_st = localtime(&myTime.tv_sec);

    ch = strftime(iso_time, sizeof(iso_time)-1,"%FT%T", time_st);
    ch = strftime(time_zone, sizeof(time_zone)-1,"%z", time_st);

    sprintf(dest, "%s.%03lu%s", iso_time, (myTime.tv_usec+500)/1000, time_zone);

    std::string time_str = dest;

    return( time_str );
}
