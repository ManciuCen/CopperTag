#ifndef _AAMED_H_
#define _AAMED_H_

#include "aamed/aamedParam.h"
#include "aamed/aamedLinkMatrix.h"
#include "aamed/aamedEllipseConstraint.h"
#include "aamed/aamedNode_FC.h"

#include "common/common.h"
#include "coppertag/tagDetectorParams.h"

#include <vector>
#include <opencv2/opencv.hpp>

typedef GroupPart<SORTDATA> GPSD;

struct EdgeSegmentYXParam {
  // the start point of current segment (first element of points)
  cv::Point startPoint;

  // the end point of current segment (final element of points)
  cv::Point endPoint;

  // whether the segment is closed
  bool isClosed;

  // the length of current segment
  int length;

  // the approximate m (y = mx + b) in line-like segment
  double m;

  // the approximate b (y = mx + b) in line-like segment
  double b;

  /**
   * @brief constructor with inputPoints parameter
   * 
   * @param inputPoints
   */
  EdgeSegmentYXParam(const std::vector<cv::Point>& inputPoints);
}; // struct EdgeSegmentYXParam

struct GetSegmentsRes {
  // original segment points, represent by (y,x)
  std::vector<std::vector<cv::Point>> points;

  // cluster segment groups
  // [0]->(0, 2, 3): means 0, 2, 3 segments in points are within the same group (probably)
  std::vector<std::vector<int>> clusters;
}; // struct GetSegmentsRes

class AAMED {
  public:
    /**
     *@brief constructor with drows and dcols parameter
     */
    AAMED(int drows, int dcols);

    /**
     *@brief destructor 
     */
    ~AAMED();

    /**
     * @brief Set the parameters
     *
     * @param theta_fsa 
     * @param length_fsa 
     * @param T_val 
     */
    void set_parameters(double theta_fsa, double length_fsa, double T_val);

    /**
     * @brief detect detect ellipse
     * 
     * @param imgEdge 
     * @param edgeSegments
     * 
     * @return candidate ellipses
     */
    std::vector<cv::RotatedRect> detect_ellipse(cv::Mat &imgEdge, std::vector<std::vector<cv::Point>> edgeSegments); 

    /**
     * @brief Get the edgeSegments object
     * 
     * @param imgInput 
     * @param imgEdge 
     * @param gradX 
     * @param gradY
     * 
     * @return original segments and clusters
     */
    GetSegmentsRes get_edgeSegments(const cv::Mat imgInput, cv::Mat &imgEdge, cv::Mat &gradX, cv::Mat &gradY); 

  private:
    /**
     * @brief determine whether the segment1 and segment2 are the same group
     * 
     * @param seg1
     * @param seg2
     * @param segParam1
     * @param segParam2
     * @param status(output)
     * 
     * @return true(are the same group) / false(not the same group)
     */
    bool segments_are_same_group(const std::vector<cv::Point>& seg1, const std::vector<cv::Point>& seg2,
                                 const EdgeSegmentYXParam& segParam1, const EdgeSegmentYXParam& segParam2, int& status);

    /**
     * @brief cluster or remove the input segments into different groups
     * 
     * @param edgeSegmentsIn
     * @param edgeSegmentParamsIn
     * @param clusterRes(output)
     */
    void segments_filter(const std::vector<std::vector<cv::Point>> &edgeSegmentsIn,
                         const std::vector<EdgeSegmentYXParam> &edgeSegmentParamsIn,
                         std::vector<std::vector<int>> &clusterRes);

    int dROWS, dCOLS;
    int iROWS, iCOLS;

    // Ellipse Fitting Varaies
    cv::Mat dls_C, dls_D, dls_V, dls_D2, dls_V2, dls_X;

    std::vector<ArcSearchRegion> asrs;

    cv::Mat source_arcs, indices_arcs, dist_arcs;

    //Validation Data
    float vldBaseData[VALIDATION_NUMBER][2];
    float vldBaseDataX[VALIDATION_NUMBER], vldBaseDataY[VALIDATION_NUMBER];
    float sample_x[VALIDATION_NUMBER], sample_y[VALIDATION_NUMBER];
    float grad_x[VALIDATION_NUMBER], grad_y[VALIDATION_NUMBER];
    float sample_weight[VALIDATION_NUMBER];

    cv::flann::LinearIndexParams indexParams;
    cv::flann::Index kdTree_Arcs;

    // The part of arc adjacency matrix (AAM)
    GroupPart<char> LinkMatrix;
    std::vector< float > query_arcs;

    struct linkArc
    {
      int idx;
      std::vector<int> idx_linking;
      std::vector<int> idx_linked;
      std::vector<int> idx_notlink;
      void clear();
      int find_next_linking(int idx_start);
      int find_next_linked(int idx_start);
    };
    std::vector<linkArc> lA;

    // The ith value represent whether ith arc is visited or not.
    GroupPart<unsigned char> visited;
    GroupPart<unsigned char> searched;

    std::vector<cv::Vec<double, MAT_NUMBER>> ArcFitMat;

    // If the value of ith is 1, the ith arc has been combined in an arc combination
    std::vector<unsigned char> _arc_grouped_label;

    std::vector<std::vector<int>> search_group[2];
    std::vector<cv::Vec<double, MAT_NUMBER>> search_arcMats[2];

    // Extracting the arc combinations according to the adjacency matrix 
    std::vector<int> temp;
    cv::Vec<double, MAT_NUMBER> fitArcTemp;

    GroupPart<SORTDATA> fitComb_LR; 

    // Ellipse Output
    std::vector<cv::RotatedRect> detEllipses;
    std::vector<double> detEllipseScore;

    double _theta_fsa;
    double _length_fsa;
    double _T_val;
    double _T_dp; 
    double _T_gradnum; 
    // The list of adaptive parameters
    //Independent of image size
    double _T_edge_num;
    //Independent of image size
    double _T_min_minor;
    static double get_T_edge_num(double theta_fsa, double &_T_min_minor); 

    Node_FC *data;

    // *** Arc and Ellipse Section *** 
    /**
     * @brief aamed auto parameter free dpContour segment
     * 
     * @param edgeSegments 
     * @param dpSegments 
     */
    void aamed_poly_DP(std::vector<std::vector<cv::Point>> &edgeSegments, 
                       std::vector<std::vector<cv::Point>> &dpSegments);

    /**
     * @brief go over a dp contour and take out the FSA segments
     * 
     * @param dpContour 
     * @param FSA_ArcContours 
     */
    void check_FSA(std::vector<cv::Point> &dpContour, 
                   std::vector<std::vector<cv::Point>> &FSA_ArcContours);

    /**
     * @brief loop over 
     * 
     * @param dpSegments 
     * @param fsaSegments 
     */
    void fsa_segments(std::vector<std::vector<cv::Point>> &dpSegments, 
                      std::vector<std::vector<cv::Point>> &fsaSegments);

    /**
     * @brief sort arcs base on their length
     * 
     * @param detArcs 
     */
    void sort_arcs(std::vector<std::vector<cv::Point>> &detArcs);

    /**
     * @brief Create a Arc Search Region object
     * 
     * @param fsaarcs 
     */
    void create_arc_search_region(std::vector<std::vector<cv::Point>> &fsaarcs);

    /**
     * @brief Get the Arcs KDTrees object using cv::flan ....
     * 
     * TODO: take out kdTree_Arcs, source_arcs, indexParams, 
     * they seems to be useless as global vairables;
     * @param arcs 
     */
    void get_arc_kdTree(std::vector<std::vector<cv::Point>> &arcs);

    /**
     * @brief group correct facing arcs
     * 
     * TODO: query_arc can be move to local var; 
     * @param fsaarcs 
     */
    void arcs_grouping(std::vector<std::vector<cv::Point>> &fsaarcs);

    /**
     * @brief 
     * TODO: first time lA? if so maybe declare here? 
     * @param _linkMatrix 
     * @param arc_num 
     */
    void get_linked_arcs(const char *_linkMatrix, int arc_num);

    /**
     * @brief Calculate Arc matrix through aamed algorithm
     * 
     * @param arcfitmat 
     * @param fsa_arc 
     */
    void get_arc_matrix(std::vector<cv::Vec<double, MAT_NUMBER>> &arcfitmat, 
                        std::vector<std::vector<cv::Point>> &fsa_arc);

    /**
     * @brief aamed algo
     * 
     * @param point_idx 
     * @param fsaSegments 
     */
    void posterior_arc_search(int point_idx, std::vector<std::vector<cv::Point>> &fsaSegments);

    /**
     * @brief aamed algo
     * 
     * @param point_idx 
     * @param fsaSegments 
     */
    void anterior_arcs_search(int point_idx, std::vector<std::vector<cv::Point>> &fsaSegments); 

    /**
     * @brief aamed algo
     * 
     * @param fitComb 
     * @param fitMats 
     * @param link_group 
     * @param arc_grouped 
     * @param fsaSegments 
     * @param imgEdge 
     */
    void bi_direction_verify(GPSD &fitComb, 
                             std::vector < cv::Vec<double, MAT_NUMBER> > fitMats[2], 
                             std::vector< std::vector<int> > link_group[2], 
                             std::vector<unsigned char> &arc_grouped, 
                             std::vector<std::vector<cv::Point>> &fsaSegments, 
                             cv::Mat &imgEdge);

    /**
     * @brief fit ellipse and other constraint to check if ellipse make sense
     * 
     * @param _linkingMat 
     * @param _linkedMat 
     * @param fitres 
     * 
     * @return true / false
     */
    bool fit_constraint(double *_linkingMat, double *_linkedMat, cv::RotatedRect &fitres); 

    /**
     * @brief custom fit ellipse 
     * 
     * @param data 
     * @param error 
     * @param res 
     */
    void fit_ellipse(double * data, double & error, cv::RotatedRect &res); 

    /**
     * @brief valid ellipse through ellipse characters
     * 
     * @param res 
     * @param detScore 
     * @param imgEdge 
     * 
     * @return true / false
     */
    bool fast_validation(cv::RotatedRect &res, double *detScore, cv::Mat &imgEdge);

    /**
     * @brief 
     * TODO: write brief
     * @param fitComb 
     * @param arcMats 
     * @param s_group 
     */
    void sort_combine(GPSD &fitComb, 
                      std::vector<cv::Vec<double, MAT_NUMBER>> arcMats[2], 
                      std::vector<std::vector<int> > s_group[2]); 

    /**
     * @brief 
     * TODO: write brief
     * @param detElps 
     * @param detEllipseScore 
     */
    void cluster_ellipses(std::vector<cv::RotatedRect> &detElps, 
                          std::vector<double> &detEllipseScore);

    /**
     * @brief Turn Gray Scale image in Edge iamge
     * also put gradient values in GradX and GradY for future line direction reference; 
     * @param imgInput input grayscale image
     * @param imgEdge output cv::Mat edge image 
     */
    void convert_edge_image(const cv::Mat imgInput, cv::Mat &imageEdge); 

    /**
     * @brief get edge segments from a edge image (AAMED)
     * uses trace_pixel() on all non-zero pixel; 
     * @param imgEdge input edge image
     * @param edgeSegments the segments
     */
    void draw_edge_contours(cv::Mat &imgEdge,
                            std::vector<std::vector<cv::Point>> &edgeSegments,
                            std::vector<EdgeSegmentYXParam> &edgeSegmentsForCluster);

    /**
     * @brief AAMED trace FindContour 
     * Tracing part of draw_edge_contours(); 
     * Move left from the given pixel to the end and also do the other direction
     * connect both to a whole segments
     * @param Wise 
     * @param antiWise 
     * @param Edge 
     * @param x 
     * @param y 
     * @param edgeSegments
     * @param edgeSegmentsForCluster
     */
    void trace_pixel(const int Wise[8][2], const int antiWise[8][2], 
                     uchar *Edge, int x, int y,
                     std::vector<std::vector<cv::Point>> &edgeSegments,
                     std::vector<EdgeSegmentYXParam> &edgeSegmentsForCluster);  

    /**
     * @brief Rewrite information in the _edge string, 
     * since find contour would wipe information from that. 
     * This is for last validation step of ellipse finding; 
     * @param _edge 
     * @param edge_Contours 
     */
    void bold_edge(uchar *_edge, const std::vector<std::vector<cv::Point>> &edge_Contours);

    /**
     * @brief turn all the fsa segments to same direction for future matching
     * @param antiContour 
     */
    void clockwise_contour(std::vector<cv::Point> &antiContour);
    // *** End of Arc and Ellipse Section *** 

    // *** Arc Grouping Helper Functions ***
    /**
     * @brief dont touch, just use... 
     */
    protected:
      char Group4FAnB1_FBmA1(cv::Point vecFet[8], const cv::Point * const *l1, const cv::Point * const *l2, const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const; 
      char Group4FAnB1_FBmA1(cv::Point vecFet[8], const cv::Point * const *l1, const cv::Point * const *l2) const; 
      char Group4CAnB1_FBmA1(cv::Point vecFet[8], const cv::Point * const *l1, const cv::Point * const *l2) const; 
      char Group4FAnB1_CBmA1(cv::Point vecFet[8], const cv::Point * const *l1, const cv::Point * const *l2) const; 
      char Group4CAnB1_CBmA1(cv::Point vecFet[8], const cv::Point * const *l1, const cv::Point * const *l2) const; 

      bool CONSTRAINT_NEIBOR_FSA(const cv::Point ni[2]) const; 
      bool CONSTRAINT_NEIBOR_TAIL(const cv::Point AB[8]) const; 
      bool CONSTRAINT_GLOBAL_REGION(const cv::Point vecFet[8], const cv::Point * const *l1, const cv::Point *checkPoint) const; 
      int CONSTRAINT_GLOABL_CURVATURE(const cv::Point vecFet[8], const int flag) const; 
      int CONSTRAINT_GLOABL_CURVATURE(const cv::Point arc_n[3]) const; 

      bool RegionConstraint(const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const; 
      bool FSAConstraint(const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const; 
      // l_ik >= T_ik && l_ki >= T_ki
      char CASE_1(const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const; 
      // l_ik >= T_ik && l_ki < T_ki
      char CASE_2(const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const; 
    // *** End of Arc Grouping Helper Functions ***

}; // class AAMED

#endif // _AAMED_H_