//
// Created by zzh on 2019/12/30.
//

#ifndef MAP_TRANSPOSE_MAP_TRANSPOSE_H
#define MAP_TRANSPOSE_MAP_TRANSPOSE_H
#include <vi-map/vi-map.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
static int bit_pattern_31_[256*4] =
        {
                8,-3, 9,5/*mean (0), correlation (0)*/,
                4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
                -11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
                7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
                2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
                1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
                -2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
                -13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
                -13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
                10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
                -13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
                -11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
                7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
                -4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
                -13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
                -9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
                12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
                -3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
                -6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
                11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
                4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
                5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
                3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
                -8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
                -2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
                -13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
                -7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
                -4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
                -10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
                5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
                5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
                1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
                9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
                4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
                2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
                -4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
                -8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
                4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
                0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
                -13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
                -3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
                -6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
                8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
                0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
                7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
                -13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
                10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
                -6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
                10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
                -13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
                -13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
                3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
                5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
                -1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
                3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
                2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
                -13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
                -13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
                -13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
                -7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
                6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
                -9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
                -2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
                -12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
                3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
                -7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
                -3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
                2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
                -11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
                -1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
                5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
                -4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
                -9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
                -12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
                10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
                7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
                -7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
                -4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
                7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
                -7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
                -13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
                -3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
                7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
                -13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
                1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
                2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
                -4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
                -1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
                7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
                1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
                9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
                -1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
                -13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
                7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
                12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
                6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
                5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
                2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
                3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
                2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
                9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
                -8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
                -11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
                1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
                6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
                2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
                6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
                3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
                7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
                -11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
                -10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
                -5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
                -10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
                8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
                4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
                -10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
                4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
                -2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
                -5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
                7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
                -9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
                -5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
                8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
                -9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
                1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
                7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
                -2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
                11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
                -12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
                3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
                5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
                0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
                -9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
                0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
                -1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
                5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
                3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
                -13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
                -5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
                -4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
                6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
                -7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
                -13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
                1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
                4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
                -2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
                2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
                -2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
                4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
                -6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
                -3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
                7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
                4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
                -13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
                7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
                7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
                -7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
                -8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
                -13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
                2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
                10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
                -6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
                8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
                2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
                -11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
                -12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
                -11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
                5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
                -2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
                -1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
                -13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
                -10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
                -3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
                2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
                -9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
                -4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
                -4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
                -6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
                6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
                -13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
                11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
                7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
                -1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
                -4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
                -7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
                -13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
                -7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
                -8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
                -5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
                -13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
                1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
                1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
                9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
                5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
                -1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
                -9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
                -1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
                -13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
                8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
                2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
                7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
                -10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
                -10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
                4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
                3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
                -4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
                5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
                4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
                -9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
                0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
                -12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
                3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
                -10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
                8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
                -8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
                2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
                10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
                6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
                -7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
                -3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
                -1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
                -3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
                -8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
                4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
                2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
                6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
                3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
                11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
                -3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
                4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
                2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
                -10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
                -13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
                -13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
                6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
                0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
                -13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
                -9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
                -13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
                5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
                2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
                -1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
                9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
                11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
                3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
                -1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
                3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
                -13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
                5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
                8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
                7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
                -10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
                7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
                9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
                7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
                -1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
        };
const float factorPI = (float)(CV_PI/180.f);
static void computeOrbDescriptor(const cv::KeyPoint& kpt,
                                 const cv::Mat& img, const cv::Point* pattern,
                                 uchar* desc)
{
    float angle = (float)kpt.angle*factorPI;
    float a = (float)cos(angle), b = (float)sin(angle);

    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
    const int step = (int)img.step;

#define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]


    for (int i = 0; i < 32; ++i, pattern += 16)
    {
        int t0, t1, val;
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = (uchar)val;
    }

}
static void computeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                               const std::vector<cv::Point>& pattern)
{
    descriptors = cv::Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

    for (size_t i = 0; i < keypoints.size(); i++)
        computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
}

void save_keyframe(const vi_map::VIMap &map,const std::string& path){
    std::fstream file_bin;
    file_bin.open(path.c_str(), std::ios_base::out|std::ios::binary|std::ios::app);
    if (!file_bin.is_open()){
        return;
    }
    std::vector<cv::Point> pattern;
    const int npoints = 512;
    const cv::Point* pattern0 = (const cv::Point*)bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));


    vi_map::MissionIdList missionids;
    map.getAllMissionIds(&missionids);
    vi_map::MissionId& first_mission = missionids.front();
    pose_graph::VertexIdList vertex_ids;
    map.getAllVertexIdsInMissionAlongGraph(first_mission, &vertex_ids);
    for (int i=0; i < vertex_ids.size(); i++){
        pose_graph::VertexId& cur_vertex_id = vertex_ids.at(i);
        const vi_map::Vertex cur_vertex = map.getVertex(cur_vertex_id);
        //TODO
        long unsigned int mnId = cur_vertex_id.hashToSizeT();
        file_bin.write((char*)&mnId, sizeof(mnId));
        if (i != 0){
            long unsigned int nNextId = vertex_ids.at(i-1).hashToSizeT();
            file_bin.write((char*)&nNextId, sizeof(nNextId));
        } else{
            long unsigned int nNextId =0;
            file_bin.write((char*)&nNextId, sizeof(nNextId));
        }
        long unsigned int mnFrameId = 0;
        file_bin.write((char*)&mnFrameId, sizeof(mnFrameId));
        double timeStamp = ((double)cur_vertex.getVisualNFrame().getFrame(0u).getTimestampNanoseconds())/(1e9);
        file_bin.write((char*)&timeStamp, sizeof(timeStamp));

        int mnGridCols =48, mnGridRows=64;
        file_bin.write((char*)&mnGridCols, sizeof(mnGridCols));
        file_bin.write((char*)&mnGridRows, sizeof(mnGridRows));
        long unsigned int mfGridElementWidthInv=0, mfGridElementHeightInv=0;
        file_bin.write((char*)&mfGridElementWidthInv,sizeof(mfGridElementWidthInv));
        file_bin.write((char*)&mfGridElementHeightInv,sizeof(mfGridElementHeightInv));

        long unsigned int mnTrackReferenceForFrame=0, mnFuseTargetForKF=0;
        long unsigned int mnBALocalForKF=0, mnBAFixedForKF=0;
        file_bin.write((char*)&mnTrackReferenceForFrame,sizeof(mnTrackReferenceForFrame));//(long unsigned int)
        file_bin.write((char*)&mnFuseTargetForKF,sizeof(mnFuseTargetForKF)); //标记在局部建图线程中，和那个关键帧进行融合的操作(long unsigned int)
        file_bin.write((char*)&mnBALocalForKF,sizeof(mnBALocalForKF));
        file_bin.write((char*)&mnBAFixedForKF,sizeof(mnBAFixedForKF));//记录触发优化的关键帧ID(long unsigned int)

        long unsigned int mnLoopQuery=0;
        int mnLoopWords=0;
        float mLoopScore=0;
        long unsigned int mnRelocQuery=0;
        int mnRelocWords=0;
        float mRelocScore=0;
        long unsigned int mnBAGlobalForKF=0;
        file_bin.write((char*)&mnLoopQuery,sizeof(mnLoopQuery));//标志当前关键帧是ID为mnLoopQuery的候选关键帧(long unsigned int)
        file_bin.write((char*)&mnLoopWords,sizeof(mnLoopWords));//具有相同word个数(int)
        file_bin.write((char*)&mLoopScore,sizeof(mLoopScore));// 匹配评分(float)
        file_bin.write((char*)&mnRelocQuery,sizeof(mnRelocQuery));//辅助重定位是要定位的帧ID(l u i)
        file_bin.write((char*)&mnRelocWords,sizeof(mnRelocWords));//单词数量(i)
        file_bin.write((char*)&mRelocScore,sizeof(mRelocScore));//匹配得分(float)
        file_bin.write((char*)&mnBAGlobalForKF,sizeof(mnBAGlobalForKF));//记录由哪一帧触发的全局BA(l u i)

        Eigen::Vector4d cam_proj = cur_vertex.getNCameras()->getCameraShared(0u)->getParameters();
        float fx = cam_proj[0];
        float fy = cam_proj[1];
        float cx = cam_proj[2];
        float cy = cam_proj[3];
        float invfx = 1/fx;
        float invfy = 1/fy;
        float mbf =0;
        float mb =0;
        float mThDepth=0;
        file_bin.write((char*)&fx,sizeof(fx));
        file_bin.write((char*)&fy,sizeof(fy));
        file_bin.write((char*)&cx,sizeof(cx));
        file_bin.write((char*)&cy,sizeof(cy));
        file_bin.write((char*)&invfx,sizeof(invfx));
        file_bin.write((char*)&invfy,sizeof(invfy)); //字面意思(float)
        file_bin.write((char*)&mbf,sizeof(mbf));
        file_bin.write((char*)&mb,sizeof(mb)); //相机基线(float)
        file_bin.write((char*)&mThDepth,sizeof(mThDepth));

        Eigen::Matrix2Xd keypoints = cur_vertex.getVisualNFrame().getFrame(0u).getKeypointMeasurements();
        cv::Mat RawImg;
        map.getRawImage(cur_vertex, 0u, &RawImg);
        int N = keypoints.cols();
        file_bin.write((char*)&N, sizeof(N));

        cv::Ptr<cv::DescriptorExtractor> descriptorExc = cv::ORB::create();
        std::vector<cv::KeyPoint> _keypoints;
        for (int j=0; j < keypoints.cols(); j++){
            cv::KeyPoint keypoint;
            keypoint.pt.x = keypoints(0,j);
            keypoint.pt.y = keypoints(1,j);
            _keypoints.emplace_back(keypoint);
        }
        cv::Mat _descriptors;
        _descriptors.create(N, 32, CV_8U);
        computeDescriptors(RawImg, _keypoints, _descriptors, pattern);

        int mnScaleLevels=0;
        float mfScaleFactor=0, mfLogScaleFactor=0;
        int mnMinX=0, mnMinY=0, mnMaxX=0, mnMaxY=0;
        bool mbFirstConnection=0, mbNotErase=0, mbToBeErased=0;
        bool mbBad=0, mHalfBaseline=0;
        file_bin.write((char*)&mnScaleLevels,sizeof(mnScaleLevels));
        file_bin.write((char*)&mfScaleFactor,sizeof(mfScaleFactor));
        file_bin.write((char*)&mfLogScaleFactor,sizeof(mfLogScaleFactor));
        file_bin.write((char*)&mnMinX,sizeof(mnMinX));
        file_bin.write((char*)&mnMinY,sizeof(mnMinY));
        file_bin.write((char*)&mnMaxX,sizeof(mnMaxX));
        file_bin.write((char*)&mnMaxY,sizeof(mnMaxY));
        file_bin.write((char*)&mbFirstConnection,sizeof(mbFirstConnection));
        file_bin.write((char*)&mbNotErase,sizeof(mbNotErase));
        file_bin.write((char*)&mbToBeErased,sizeof(mbToBeErased));
        file_bin.write((char*)&mbBad,sizeof(mbBad));
        file_bin.write((char*)&mHalfBaseline,sizeof(mHalfBaseline));

        Eigen::Quaterniond q = cur_vertex.get_q_M_I();
        Eigen::Matrix3d R_M_I = q.toRotationMatrix();
        Eigen::Vector3d t_M_I = cur_vertex.get_p_M_I();
        Eigen::Isometry3d T_M_I;
        T_M_I.rotate(q);
        T_M_I.pretranslate(t_M_I);
        Eigen::Isometry3d T_I_M = T_M_I.inverse();

        bool strnull = false;
        bool strcont = true;

        file_bin.write((char*)&strnull, sizeof(strnull));//mTcwGBA
        file_bin.write(((char*)&strnull), sizeof(strnull));//mTcwBefGBA

        for (int i=0; i < _descriptors.rows; ++i){
            const unsigned char *p = _descriptors.ptr<unsigned char>(i);
            for (int j=0; j < _descriptors.cols; ++j, ++p){
                int tempintdesc=(int)*p;
                file_bin.write((char*)&tempintdesc, sizeof(tempintdesc));
            }
        }

        file_bin.write((char*)&strnull, sizeof(strnull));//mTcp
        file_bin.write((char*)&strnull, sizeof(strnull));//mK
        //TODO
        file_bin.write((char*)&strnull, sizeof(strnull));//Tcw
        float b11=T_I_M(0,0), b12=T_I_M(0,1),b13=T_I_M(0,2), b14=T_I_M(0,3),
                b21=T_I_M(1,0), b22=T_I_M(1,1), b23=T_I_M(1,2), b24=T_I_M(1,3),
                b31=T_I_M(2,0), b32=T_I_M(2,1), b33=T_I_M(2,2), b34=T_I_M(2,3),
                b41=T_I_M(3,0), b42=T_I_M(3,1), b43=T_I_M(3,2), b44=T_I_M(3,3);
        file_bin.write((char*)&b11, sizeof(b11));
        file_bin.write((char*)&b12, sizeof(b12));
        file_bin.write((char*)&b13, sizeof(b13));
        file_bin.write((char*)&b14, sizeof(b14));
        file_bin.write((char*)&b21, sizeof(b21));
        file_bin.write((char*)&b22, sizeof(b22));
        file_bin.write((char*)&b23, sizeof(b23));
        file_bin.write((char*)&b24, sizeof(b24));
        file_bin.write((char*)&b31, sizeof(b31));
        file_bin.write((char*)&b32, sizeof(b32));
        file_bin.write((char*)&b33, sizeof(b33));
        file_bin.write((char*)&b34, sizeof(b34));
        file_bin.write((char*)&b41, sizeof(b41));
        file_bin.write((char*)&b42, sizeof(b42));
        file_bin.write((char*)&b43, sizeof(b43));
        file_bin.write((char*)&b44, sizeof(b44));
        //file_bin.write((char*)&strnull, sizeof(strnull));//Twc
        file_bin.write((char*)&strcont, sizeof(strcont));
        float a11=T_M_I(0,0), a12=T_M_I(0,1),a13=T_M_I(0,2), a14=T_M_I(0,3),
                a21=T_M_I(1,0), a22=T_M_I(1,1), a23=T_M_I(1,2), a24=T_M_I(1,3),
                a31=T_M_I(2,0), a32=T_M_I(2,1), a33=T_M_I(2,2), a34=T_M_I(2,3),
                a41=T_M_I(3,0), a42=T_M_I(3,1), a43=T_M_I(3,2), a44=T_M_I(3,3);
        file_bin.write((char*)&a11, sizeof(a11));
        file_bin.write((char*)&a12, sizeof(a12));
        file_bin.write((char*)&a13, sizeof(a13));
        file_bin.write((char*)&a14, sizeof(a14));
        file_bin.write((char*)&a21, sizeof(a21));
        file_bin.write((char*)&a22, sizeof(a22));
        file_bin.write((char*)&a23, sizeof(a23));
        file_bin.write((char*)&a24, sizeof(a24));
        file_bin.write((char*)&a31, sizeof(a31));
        file_bin.write((char*)&a32, sizeof(a32));
        file_bin.write((char*)&a33, sizeof(a33));
        file_bin.write((char*)&a34, sizeof(a34));
        file_bin.write((char*)&a41, sizeof(a41));
        file_bin.write((char*)&a42, sizeof(a42));
        file_bin.write((char*)&a43, sizeof(a43));
        file_bin.write((char*)&a44, sizeof(a44));

        file_bin.write((char*)&strnull, sizeof(strnull));//Ow

        file_bin.write((char*)&strnull, sizeof(strnull));//Cw
        //mvKey
        for (int i=0; i < _keypoints.size(); i++){
            cv::KeyPoint point = _keypoints.at(i);
            file_bin.write((char*)&point.pt.x, sizeof(point.pt.x));
            file_bin.write((char*)&point.pt.y, sizeof(point.pt.y));
            file_bin.write((char*)&point.size, sizeof(point.size));
            file_bin.write((char*)&point.angle, sizeof(point.angle));
            file_bin.write((char*)&point.response, sizeof(point.response));
            file_bin.write((char*)&point.octave, sizeof(point.octave));
            file_bin.write((char*)&point.class_id, sizeof(point.class_id));
        }
        int mvkeysunsize=0;
        file_bin.write((char*)&mvkeysunsize, sizeof(mvkeysunsize));
        int mvurightsize=0;
        file_bin.write((char*)&mvurightsize, sizeof(mvurightsize));
        int mvdepthsize=0;
        file_bin.write((char*)&mvdepthsize, sizeof(mvdepthsize));
        int mvScaleFactors=0;
        file_bin.write((char*)&mvScaleFactors, sizeof(mvScaleFactors));
        int mvLevelSigma2=0;
        file_bin.write((char*)&mvLevelSigma2, sizeof(mvLevelSigma2));
        int mvInvLevelSigma2=0;
        file_bin.write((char*)&mvInvLevelSigma2, sizeof(mvInvLevelSigma2));
        //TODO
        int MPcounter=0;//需要重新写
        file_bin.write((char*)&MPcounter, sizeof(MPcounter));
        /*vi_map::LandmarkIdList oblandIds;
        cur_vertex.getFrameObservedLandmarkIds(0u, &oblandIds);q
        int MPcounter = oblandIds.size();
        file_bin.write((char*)&MPcounter, sizeof(MPcounter));
        for (vi_map::LandmarkId oblandId : oblandIds){
            const vi_map::Landmark& obland = map.getLandmark(oblandId);
            const vi_map::KeypointIdentifierList& keys = obland.getObservations();
            for (vi_map::KeypointIdentifier key : keys){
                if (cur_vertex_id == key.frame_id.vertex_id){
                    int i = key.keypoint_index;
                    long unsigned int M_mnID = oblandId.hashToSizeT();
                    file_bin.write((char*)&i, sizeof(i));
                    file_bin.write((char*)&M_mnID, sizeof(M_mnID));
                    break;
                }
            }
        }*/

        int m=64, n=48;
        for (int i=0; i<m; i ++)
            for (int j =0; j<n; j++){
                int layer2size = 0;
                file_bin.write((char*)&layer2size, sizeof(layer2size));
            }
        int mvpOrderedConnectedKeyFramessize=0;
        file_bin.write((char*)&mvpOrderedConnectedKeyFramessize, sizeof(mvpOrderedConnectedKeyFramessize));
        int mvOrderedWeightssize=0;
        file_bin.write((char*)&mvOrderedWeightssize, sizeof(mvOrderedWeightssize));
        int mBowVecsize=0;
        file_bin.write((char*)&mBowVecsize, sizeof(mBowVecsize));
        int mFeatVecsize=0;
        file_bin.write((char*)&mFeatVecsize, sizeof(mFeatVecsize));
        int mConnectedKeyFrameWeightssize=0;
        file_bin.write((char*)&mConnectedKeyFrameWeightssize, sizeof(mConnectedKeyFrameWeightssize));
        int mspChildrenssize=0;
        file_bin.write((char*)&mspChildrenssize, sizeof(mspChildrenssize));
        int mspLoopEdgessize=0;
        file_bin.write((char*)&mspLoopEdgessize, sizeof(mspLoopEdgessize));

        file_bin.write((char*)&strnull, sizeof(strnull));//mpParent*/
        std::cout << i << "frame save succed" << std::endl;


    }
    file_bin.close();
}

void save_mappoint(const vi_map::VIMap& map, const std::string& path){
    std::fstream file_bin;
    file_bin.open(path.c_str(), std::ios_base::out|std::ios::binary|std::ios::app);
    if (!file_bin.is_open()){
        return;
    }
    std::vector<cv::Point> pattern;
    const int npoints = 512;
    const cv::Point* pattern0 = (const cv::Point*)bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));


    vi_map::MissionIdList missionids;
    map.getAllMissionIds(&missionids);
    vi_map::MissionId& first_mission = missionids.front();
    vi_map::LandmarkIdList landmarkIds;
    map.getAllLandmarkIdsInMission(first_mission, &landmarkIds);
    for (int i=0; i <landmarkIds.size(); i++){
        const vi_map::Landmark& landmark  = map.getLandmark(landmarkIds.at(i));
        long unsigned int mnId = landmarkIds.at(i).hashToSizeT();
        file_bin.write((char*)&mnId, sizeof(mnId));
        if (i!=0){
            long unsigned int nNextId = landmarkIds.at(i-1).hashToSizeT();
            file_bin.write((char*)&nNextId, sizeof(nNextId));
        } else{
            long unsigned int nNextId =0;
            file_bin.write((char*)&nNextId, sizeof(nNextId));
        }
        long int mnFirstKFid = 0;
        long int mnFirstFrame = 0;
        file_bin.write((char*)&mnFirstKFid, sizeof(mnFirstKFid));
        file_bin.write((char*)&mnFirstFrame, sizeof(mnFirstFrame));
        int nObs = landmark.getObservations().size();
        file_bin.write((char*)&nObs, sizeof(nObs));
        float mTrackProjX=0, mTrackProjY=0, mTrackProjXR=0;
        file_bin.write((char*)&mTrackProjX, sizeof(mTrackProjX));
        file_bin.write((char*)&mTrackProjY, sizeof(mTrackProjY));
        file_bin.write((char*)&mTrackProjXR, sizeof(mTrackProjXR));

        bool mbTrackInView=0;
        file_bin.write((char*)&mbTrackInView, sizeof(mbTrackInView));
        int mnTrackScaleLevel=0;
        file_bin.write((char*)&mnTrackScaleLevel, sizeof(mnTrackScaleLevel));
        float mTrackViewCos=0;
        file_bin.write((char*)&mTrackViewCos, sizeof(mTrackViewCos));

        long unsigned int mnTrackReferenceForFrame=0, mnLastFrameSeen=0,
                mnBALocalForKF=0,  mnFuseCandidateForKF=0, mnLoopPointForKF=0,
                mnCorrectedByKF=0;
        file_bin.write((char*)&mnTrackReferenceForFrame, sizeof(mnTrackReferenceForFrame));
        file_bin.write((char*)&mnLastFrameSeen, sizeof(mnLastFrameSeen));
        file_bin.write((char*)&mnBALocalForKF, sizeof(mnBALocalForKF));
        file_bin.write((char*)&mnFuseCandidateForKF, sizeof(mnFuseCandidateForKF));
        file_bin.write((char*)&mnLoopPointForKF, sizeof(mnLoopPointForKF));
        file_bin.write((char*)&mnCorrectedByKF, sizeof(mnCorrectedByKF));

        long unsigned int mnCorrectedReference=0, mnBAGlobalForKF=0;
        file_bin.write((char*)&mnCorrectedReference, sizeof(mnCorrectedReference));
        file_bin.write((char*)&mnBAGlobalForKF, sizeof(mnBAGlobalForKF));

        int mnVisible=0, mnFound=0;
        file_bin.write((char*)&mnVisible, sizeof(mnVisible));
        file_bin.write((char*)&mnFound, sizeof(mnFound));
        bool mbBad = 0;
        file_bin.write((char*)&mbBad, sizeof(mbBad));
        float mfMinDistance=0, mfMaxDistance=0;
        file_bin.write((char*)&mfMinDistance, sizeof(mfMinDistance));
        file_bin.write((char*)&mfMaxDistance, sizeof(mfMaxDistance));

        bool strnull = false;
        bool strcont = true;

        const pose::Position3D& position = landmark.get_p_B();

        file_bin.write((char*)&strnull, sizeof(strnull));//mPosGBA

        //mWorldPos
        float x = position.x(), y = position.y(), z = position.z();
        file_bin.write((char*)&strcont, sizeof(strcont));
        file_bin.write((char*)&x, sizeof(x));
        file_bin.write((char*)&y, sizeof(y));
        file_bin.write((char*)&z, sizeof(z));

        file_bin.write((char*)&strnull, sizeof(strnull));

        //mDescriptor
        //TODO
        vi_map::KeypointIdentifierList keypointIds = landmark.getObservations();
        vi_map::KeypointIdentifier keypointId = keypointIds.front();
        pose_graph::VertexId& vId = keypointId.frame_id.vertex_id;
        const vi_map::Vertex& v = map.getVertex(vId);
        cv::Mat RawImg;
        map.getRawImage(v, 0u, &RawImg);
        size_t p_index = keypointId.keypoint_index;
        Eigen::Matrix2Xd point = v.getVisualNFrame().getFrame(0u).getKeypointMeasurement(p_index);
        cv::KeyPoint _point;
        _point.pt.x = point(0, 0);
        _point.pt.y = point(1,0);
        std::vector<cv::KeyPoint> _points;
        _points.emplace_back(_point);
        cv::Mat _descriptor;
        _descriptor.create(1,32,CV_8U);
        computeDescriptors(RawImg, _points, _descriptor, pattern);
        const unsigned char *p = _descriptor.ptr<unsigned char>();
        for (int i=0; i<_descriptor.cols; ++i, ++p){
            int tempindesc = (int)*p;
            file_bin.write((char*)&tempindesc, sizeof(tempindesc));
        }

        int mObservationssize = keypointIds.size();
        file_bin.write((char*)&mObservationssize, sizeof(mObservationssize));
        for (int i=0; i < keypointIds.size(); i++){
            vi_map::KeypointIdentifier keypointId = keypointIds.at(i);
            size_t mitsecond = keypointId.keypoint_index;
            pose_graph::VertexId obVertexId = keypointId.frame_id.vertex_id;
            long unsigned int K_mnId = obVertexId.hashToSizeT();
            file_bin.write((char*)&K_mnId, sizeof(K_mnId));
            file_bin.write((char*)&mitsecond, sizeof(mitsecond));
        }

        file_bin.write((char*)&strnull, sizeof(strnull)); //mpRefKF
        file_bin.write((char*)&strnull, sizeof(strnull)); //mpReplaced

        std::cout << i << "landmark save succeed" << std::endl;
    }
    file_bin.close();
    return;
}
#endif //MAP_TRANSPOSE_MAP_TRANSPOSE_H
