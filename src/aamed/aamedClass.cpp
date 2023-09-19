#include "aamed/AAMED.h"
#include "common/common.h"

EdgeSegmentYXParam::EdgeSegmentYXParam(const std::vector<cv::Point>& inputPoints) {
  startPoint = inputPoints[0];
  endPoint = inputPoints[inputPoints.size() - 1];
  length = distance_of_2point(startPoint, endPoint);
  if (length < 5) {
    isClosed = true;
    length = 0;
  } else {
    isClosed = false;
  }
  std::pair<double, double> mb = find_line_mb(startPoint, endPoint);
  m = mb.first; 
  b = mb.second; 
}

AAMED::AAMED(int drows, int dcols)
{

  dROWS = drows; dCOLS = dcols;
  iROWS = drows; iCOLS = dcols; 

  data = new Node_FC[dROWS*dCOLS];
  for (int i = 0; i < dROWS; i++){
    for (int j = 0; j < dCOLS; j++){
      data[dIDX(i, j)] = Node_FC(i, j, PIXEL_SCALE);
    }
  }

  _T_dp = sqrt(2.0);
  _T_gradnum = 5;
  
  dls_C = cv::Mat::zeros(6, 6, CV_64FC1);
  dls_C.at<double>(2, 2) = 1;


  double h = CV_PI * 2 / VALIDATION_NUMBER;
  for (int i = 0; i < VALIDATION_NUMBER; i++)
  {
    vldBaseData[i][0] = cos(i*h);
    vldBaseData[i][1] = sin(i*h);
    vldBaseDataX[i] = cos(i*h);
    vldBaseDataY[i] = sin(i*h);
  }

  LinkMatrix.Update(800 * 800);
  query_arcs.resize(2);
  // lA.resize(800); // Maybe no need?

  // visited.Update(800); // Maybe no need?
  fitComb_LR.Update(512);
  asrs.reserve(800);
  ArcFitMat.reserve(512);
  // ///////Pre-Reserver////////////////////////

  detEllipses.clear();
  detEllipseScore.clear();

}

AAMED::~AAMED() {
  LinkMatrix.release();
  fitComb_LR.release();
  delete[] data;
}

void AAMED::linkArc::clear()
{
  idx_linking.clear();
  idx_linked.clear();
  idx_notlink.clear();
}
int AAMED::linkArc::find_next_linking(int idx_start)
{
  int linking_num = (int)idx_linking.size();
  return idx_start + 1 < linking_num ? idx_start + 1 : -1;
}
int AAMED::linkArc::find_next_linked(int idx_start)
{
  int linked_num = (int)idx_linked.size();
  return idx_start + 1 < linked_num ? idx_start + 1 : -1;
}


std::vector<cv::RotatedRect> AAMED::detect_ellipse(
  cv::Mat &imgEdge, 
  std::vector<std::vector<cv::Point>> edgeSegments)
{
  detEllipses.clear(); 
  detEllipseScore.clear();


  std::vector<std::vector<cv::Point>> aamedDpSegents; 
  aamed_poly_DP(edgeSegments, aamedDpSegents);


  std::vector<std::vector<cv::Point>> fsaSegments;
  fsa_segments(aamedDpSegents, fsaSegments);
  
  // std::cout << "fsaSegments Size: " << fsaSegments.size() << std::endl;

  sort_arcs(fsaSegments);

  // Can Return If No FSA Segments; 
  if(fsaSegments.size() == 0){
    detEllipses.clear(); 
    return detEllipses;
  }
  create_arc_search_region(fsaSegments);

  int fsa_Arcs = int(fsaSegments.size());

  get_arc_kdTree(fsaSegments);


  arcs_grouping(fsaSegments);



  const char *_linkMatrix = LinkMatrix.GetDataPoint();
  get_linked_arcs(_linkMatrix, fsa_Arcs);

  visited.Update(fsa_Arcs);
  searched.Update(fsa_Arcs);


  get_arc_matrix(ArcFitMat, fsaSegments); //计算每个弧段的拟合矩阵

  _arc_grouped_label.clear();
  _arc_grouped_label.resize(fsa_Arcs, 0);


  for (int root_idx = 0; root_idx < fsa_Arcs; root_idx++)
  {
    if (fsaSegments[root_idx].size() <= 3)
      continue;

    if (_arc_grouped_label[root_idx] != 0)
      continue;
      
    search_group[0].clear(), search_group[1].clear();//清空上一租数据点
    search_arcMats[0].clear(), search_arcMats[1].clear(); //清空上一组矩阵拟合数据点
    searched.clean();

    temp.push_back(root_idx);//加入根节点弧段
    *visited[root_idx] = 1; //当前节点访问过
    memcpy(fitArcTemp.val, ArcFitMat[root_idx].val, sizeof(double)*MAT_NUMBER);

    posterior_arc_search(root_idx, fsaSegments);
    
    temp.push_back(root_idx);
    *visited[root_idx] = 1;
    anterior_arcs_search(root_idx, fsaSegments);

    sort_combine(fitComb_LR, search_arcMats, search_group);
    
    bi_direction_verify(fitComb_LR, search_arcMats, 
      search_group, _arc_grouped_label, fsaSegments, imgEdge);
  }

  cluster_ellipses(detEllipses, detEllipseScore);
  asrs.clear();

  return detEllipses; 
}

GetSegmentsRes AAMED::get_edgeSegments(const cv::Mat imgInput, cv::Mat &imgEdge, cv::Mat &gradX, cv::Mat &gradY)
{
  GetSegmentsRes curRes;

  // step-1. convert the input image from gray scale(or binary image) to edge image using Canny method  
  convert_edge_image(imgInput, imgEdge);

  // step-2. trace and collect all non-zero pixel to one group as a segment
  std::vector<EdgeSegmentYXParam> edgeSegmentsYXForCluster;
  draw_edge_contours(imgEdge, curRes.points, edgeSegmentsYXForCluster);

  // step-3. group some real close segments to a cluster for post-processing
  // std::vector<std::vector<int>> clusterResDSU;
  // segments_filter(curRes.points, edgeSegmentsYXForCluster, curRes.clusters);

  // step-4. used by AAMED
  bold_edge(imgEdge.data, curRes.points);

  // step-5. calculate the input image x-gradient and y-gradient for line direction correction
  cv::Sobel(imgInput, gradX, CV_16S, 1, 0, 3);
  cv::Sobel(imgInput, gradY, CV_16S, 0, 1, 3);
  
  return curRes;
}