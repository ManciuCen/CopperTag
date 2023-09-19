// #include "tagDetector.h"
#include "aamed/AAMED.h"
//搜索准则，输入point_idx 时，说明是以这个根节点为基准，查找下一个满足条件的弧段
// namespace CopperTag{
	using namespace cv;
	using namespace std;
	
	void init_fitmat(double *basic_fitmat, double *init_fitmat, int num)
	{
		for (int i = 0; i < num; i++)
			basic_fitmat[i] = init_fitmat[i];
	}

	void push_fitmat(double *basic_fitmat, double *add_fitmat, int num)
	{
		for (int i = 0; i < num; i++)
			basic_fitmat[i] += add_fitmat[i];
	}

	void pop_fitmat(double *basic_fitmat, double *minus_fitmat, int num)
	{
		for (int i = 0; i < num; i++)
			basic_fitmat[i] -= minus_fitmat[i];
	}

	void AAMED::posterior_arc_search(int point_idx, 
		std::vector<std::vector<cv::Point>> &fsaSegments)
	{
		const int arcs_num = fsaSegments.size(), group_num = temp.size();
		char *_link_data = LinkMatrix.GetDataPoint();
		int idx_use;
		bool isValid = true, noArcs = false;
		ArcSearchRegion *_asrs_data = asrs.data();

		char check_val, temp_val;
		int find_now_idx = -1, find_arc_idx;
		vector<Point> *single_arc = NULL;


		//创建当前组合弧段的搜索区间
		Point *_st_data(NULL), *_ed_data(NULL);
		int grouped_st_idx, grouped_ed_idx, grouped_ed_arc_num;
		ArcSearchRegion grouped_arcs;

		grouped_st_idx = temp[0], grouped_ed_idx = temp.back();
		_st_data = fsaSegments[grouped_st_idx].data();
		grouped_ed_arc_num = fsaSegments[grouped_ed_idx].size();
		_ed_data = fsaSegments[grouped_ed_idx].data();
		grouped_arcs.create(_st_data, _st_data + 1, _ed_data + grouped_ed_arc_num - 2, _ed_data + grouped_ed_arc_num - 1);

		while (1)
		{
			find_now_idx = lA[point_idx].find_next_linking(find_now_idx);
			if (find_now_idx == -1) //再也找不到可以组合的弧段
			{
				break;
			}
			find_arc_idx = lA[point_idx].idx_linking[find_now_idx]; //获取弧段角标
			if (*visited[find_arc_idx] != 0) //访问过
				continue;
			if (_arc_grouped_label[find_arc_idx] != 0) //被组合过
				continue;

			*visited[find_arc_idx] = 1;
			*searched[find_arc_idx] = 1;
			//验证候选组合点的中心点是否在搜索区间内(考虑到首尾均可能与当前区域相连，只有中点不会造成区域约束的实效).
			int single_num = fsaSegments[find_arc_idx].size() / 2;
			bool inSearch = grouped_arcs.is_in_search_region(&(fsaSegments[find_arc_idx][single_num]));
			if (!inSearch)
				continue;

			// 验证是否满足第二个约束式
			idx_use = find_arc_idx*arcs_num;
			isValid = true;
			for (int i = 0; i < group_num; i++)
			{
				int tmpData_i = temp[i];
				check_val = _link_data[idx_use + tmpData_i];
				if (check_val == 0) // L_{i,k} ==0 ,there needs to use CASE_1 to varify whether L_{i,k} = -1.
				{
					temp_val = CASE_1(_asrs_data + point_idx, _asrs_data + tmpData_i);
					if (temp_val == -1)
					{
						_link_data[idx_use + tmpData_i] = _link_data[tmpData_i * arcs_num + point_idx] = -1;
						isValid = false;
						break;
					}
				}
				else if (check_val == -1)
				{
					isValid = false;
					break;
				}
			}
			if (isValid == false)
				continue;


			// 这个点是有效的
			temp.push_back(find_arc_idx);
			push_fitmat(fitArcTemp.val, ArcFitMat[find_arc_idx].val, MAT_NUMBER);
			posterior_arc_search(find_arc_idx, fsaSegments);
		}

		//到这说明没有弧段可以组合了，获得当前组合，然后退掉一个点
		search_group[0].push_back(temp); //得到一个组合
		search_arcMats[0].push_back(fitArcTemp); //存储对应组合的拟合矩阵

		pop_fitmat(fitArcTemp.val, ArcFitMat[temp.back()].val, MAT_NUMBER); //删掉最后一个点的拟合矩阵
		temp.pop_back();
		*visited[point_idx] = 0;
	}
	
	void AAMED::anterior_arcs_search(int point_idx, 
		std::vector<std::vector<cv::Point>> &fsaSegments)
	{
		const int arcs_num = fsaSegments.size(), group_num = temp.size();
		char *_link_data = LinkMatrix.GetDataPoint();
		int idx_use = point_idx*arcs_num;
		bool isValid = true;

		ArcSearchRegion *_asrs_data = asrs.data();
		int *_temp_data = temp.data();
		char check_val, temp_val;





		//检查当前点与其他组合点是否可相连 Linking Search the 2th search constraint
		for (int i = 0; i < group_num - 1; i++)
		{
			check_val = _link_data[idx_use + _temp_data[i]];
			if (check_val == 0)
			{
				temp_val = CASE_1(_asrs_data + point_idx, _asrs_data + _temp_data[i]);
				if (temp_val == -1)
				{
					_link_data[idx_use + _temp_data[i]] = _link_data[_temp_data[i] * arcs_num + point_idx] = -1;
					isValid = false;
					break;
				}
			}
			else if (check_val == -1)
			{
				isValid = false;
				break;
			}
		}
		if (/*temp.size() > MAX_COMBINATION_ARCS ||*/ isValid == false)
		{
			search_group[1].push_back(temp); //得到一个组合
			search_arcMats[1].push_back(fitArcTemp); //存储对应组合的拟合矩阵

			pop_fitmat(fitArcTemp.val, ArcFitMat[temp.back()].val, MAT_NUMBER); //删掉最后一个点的拟合矩阵
			temp.pop_back();

			*visited[point_idx] = 0;
			return;
		}

		int find_now_idx = -1;
		while (1)
		{
			find_now_idx = lA[point_idx].find_next_linked(find_now_idx);
			if (find_now_idx < 0) //搜不到新点
			{
				search_group[1].push_back(temp);
				search_arcMats[1].push_back(fitArcTemp);

				pop_fitmat(fitArcTemp.val, ArcFitMat[temp.back()].val, MAT_NUMBER); //删掉最后一个点的拟合矩阵
				temp.pop_back();
				*visited[point_idx] = 0;
				break;
			}
			int find_arc_idx = lA[point_idx].idx_linked[find_now_idx];


			if (*visited[find_arc_idx] != 0)
				continue;
			if (_arc_grouped_label[find_arc_idx] != 0)
				continue;
			if (*searched[find_arc_idx] != 0)
				continue;
			*visited[find_arc_idx] = 1;
			push_fitmat(fitArcTemp.val, ArcFitMat[find_arc_idx].val, MAT_NUMBER);
			temp.push_back(find_arc_idx);
			anterior_arcs_search(find_arc_idx, fsaSegments);
		}
	}


	void AAMED::bi_direction_verify(GPSD &fitComb, 
		vector < cv::Vec<double, MAT_NUMBER> > fitMats[2], 
		vector< vector<int> > link_group[2], vector<unsigned char> &arc_grouped, 
		std::vector<std::vector<cv::Point>> &fsaSegments, 
  	cv::Mat &imgEdge)
	{

		int fit_num = fitComb.usesize(), idx_l, idx_r, num_idx_l, num_idx_r;
		cv::RotatedRect fitelpres;

		bool isCombValid = true;
		int linked_num, linking_num, *_linked_data(NULL), *_linking_data(NULL), idx_link;
		char *_link_data = LinkMatrix.GetDataPoint();
		int arcs_num = fsaSegments.size();


		int max_idx(-1), max_idx_l, max_idx_r;
		double max_score(-1);
		cv::RotatedRect max_ellipse;

		for (int i = 0; i < fit_num; i++)
		{
			if (fitComb[i]->val < 0) //判断是否有效
				break;
			bool fitres;
			idx_l = fitComb[i]->idx_l; idx_r = fitComb[i]->idx_r;
			if (idx_l >= 0 && idx_r >= 0)
			{
				linked_num = link_group[1][idx_l].size(), linking_num = link_group[0][idx_r].size();
				_linked_data = link_group[1][idx_l].data();
				_linking_data = link_group[0][idx_r].data();
				for (int k = 0; k < linked_num; k++)
				{
					idx_link = _linked_data[k] * arcs_num;
					for (int p = 0; p < linking_num; p++)
					{
						if (_link_data[idx_link + _linking_data[p]] == -1)
						{
							isCombValid = false;
							break;
						}
					}
					if (isCombValid == false)
						break;
				}
				if (isCombValid == false)//两个弧段不能组合
					continue;
				fitres = fit_constraint(fitMats[0][idx_r].val, fitMats[1][idx_l].val, fitelpres);


			}
			else if (idx_l >= 0)
				fitres = fit_constraint(NULL, fitMats[1][idx_l].val, fitelpres);
			else if (idx_r >= 0)
				fitres = fit_constraint(fitMats[0][idx_r].val, NULL, fitelpres);
				

			if (fitres == false)
				continue;

			//利用validate进行比较
			double detScore;

			fitres = fast_validation(fitelpres, &detScore, imgEdge);

			if (fitres == false)
				continue;


			num_idx_l = idx_l >= 0 ? search_group[1][idx_l].size() : -1;
			num_idx_r = idx_r >= 0 ? search_group[0][idx_r].size() : -1;
			for (int k = 0; k < num_idx_l; k++)
				arc_grouped[search_group[1][idx_l][k]] = 1;
			for (int k = 0; k < num_idx_r; k++)
				arc_grouped[search_group[0][idx_r][k]] = 1;
			detEllipses.push_back(fitelpres);
			detEllipseScore.push_back(detScore);

			break;


		}

	}


	void AAMED::sort_combine(GPSD &fitComb, 
		vector<cv::Vec<double, MAT_NUMBER>> arcMats[2], 
		vector< vector<int> > s_group[2])
	{
		double t1, t2;

		int temp_pos = 0;//统计负数个数


		int linking_num = arcMats[0].size(), linked_num = arcMats[1].size(), idx_i, idx;
		double val_l, val_r;
		fitComb.Update(linking_num*linked_num + linking_num + linked_num);
		for (int i = 0; i < linking_num; i++) //r
		{
			val_r = arcMats[0][i][MAT_NUMBER - 1];
			idx_i = i*linked_num;
			for (int j = 0; j < linked_num; j++)
			{
				if (s_group[1][j].size() >= 2)
				{
					val_l = arcMats[1][j][MAT_NUMBER - 1];
					fitComb[idx_i + j]->idx_r = i;
					fitComb[idx_i + j]->idx_l = j;
					fitComb[idx_i + j]->val = val_l + val_r;
				}
				else
				{
					fitComb[idx_i + j]->val = -1;
					temp_pos++;
				}
			}
		}
		idx_i = linking_num*linked_num;
		int arc_dp_idx;
		for (int i = 0; i < linking_num; i++)
		{
			idx = idx_i + i;
			
			fitComb[idx]->val = arcMats[0][i][MAT_NUMBER - 1];
			fitComb[idx]->idx_r = i;
			fitComb[idx]->idx_l = -1;
		}
		idx_i = linking_num*linked_num + linking_num;
		for (int i = 0; i < linked_num; i++)
		{
			idx = idx_i + i;
			if (s_group[1][i].size() > 2)
			{
				fitComb[idx]->val = arcMats[1][i][MAT_NUMBER - 1];
				fitComb[idx]->idx_l = i;
				fitComb[idx]->idx_r = -1;
			}
			else
			{
				fitComb[idx]->val = -1;
				temp_pos++;
			}
		}
		std::sort(fitComb[0], fitComb[linking_num*linked_num + linking_num + linked_num - 1], cmp);
	}


	void AAMED::cluster_ellipses(std::vector<cv::RotatedRect> &detElps, 
		vector<double> &detEllipseScore)
	{
		const int ellipse_num = (int)detElps.size();
		if (ellipse_num == 0) return;

		// The first ellipse is assigned to a cluster
		std::vector<cv::RotatedRect> clusters;
		std::vector<double> scores;
		clusters.push_back(detElps[0]);
		scores.push_back(detEllipseScore[0]);

		bool bFoundCluster = false;

		float th_Da = 0.12f;
		float th_Db = 0.12f;
		float th_Dc_ratio = 0.12f;

		//float th_Dr = 0.13f;
		float th_Dr = 30.0 / 180 * CV_PI;

		float th_Dr_circle = 0.8f;

		float loop_width = 10;

		for (int i = 1; i < ellipse_num; i++)
		{
			cv::RotatedRect& e1 = detElps[i];
			int sz_clusters = int(clusters.size());

			float ba_e1 = e1.size.height / e1.size.width;
			float Decc1 = e1.size.height / e1.size.width;
			float score_i = detEllipseScore[i];

			bool bFoundCluster = false;
			for (int j = 0; j < sz_clusters; ++j)
			{
				cv::RotatedRect& e2 = clusters[j];

				float ba_e2 = e2.size.height / e2.size.width;
				float th_Dc = std::min(e1.size.height, e2.size.height) * th_Dc_ratio;
				if (th_Dc < 30 * th_Dc_ratio) th_Dc = 30 * th_Dc_ratio;
				th_Dc *= th_Dc;

				// Centers
				float Dc = ((e1.center.x - e2.center.x)*(e1.center.x - e2.center.x) + (e1.center.y - e2.center.y)*(e1.center.y - e2.center.y));
				if (Dc > th_Dc) continue;

				// a
				float Da = abs(e1.size.width - e2.size.width) / std::max(e1.size.width, e2.size.width);
				if (Da > th_Da && abs(e1.size.width - e2.size.width) > loop_width) continue;

				// b
				float Db = abs(e1.size.height - e2.size.height) / std::min(e1.size.height, e2.size.height);
				if (Db > th_Db && abs(e1.size.height - e2.size.height) > loop_width) continue;

				// angle
				float diff = std::abs(e1.angle - e2.angle) / 180 * CV_PI;
				float Dr = diff < CV_PI / 2 ? diff : CV_PI - diff;
				if ((Dr > th_Dr) && (ba_e1 < th_Dr_circle) && (ba_e2 < th_Dr_circle))
				{
					//not same cluster
					continue;
				}
				//Score
				float score_j = scores[j];

				if (score_i > score_j) 
				{
					clusters[j] = e1;
					scores[j] = score_i;
				}
				// Same cluster as e2
				bFoundCluster = true;//
									// Discard, no need to create a new cluster
				break;
			}
			if (!bFoundCluster)
			{
				// Create a new cluster			
				clusters.push_back(e1);
				scores.push_back(score_i);
			}
		}
		clusters.swap(detElps);
		scores.swap(detEllipseScore);
	}

// }