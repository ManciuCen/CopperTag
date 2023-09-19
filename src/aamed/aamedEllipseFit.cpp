// #include "tagDetector.h"
#include "aamed/AAMED.h"

// namespace CopperTag{
  bool AAMED::fit_constraint(double *_linkingMat, 
    double *_linkedMat, cv::RotatedRect &fitres)
  {
    double fitingMat[MAT_NUMBER];
    if (_linkingMat != NULL&&_linkedMat != NULL)
    {
      for (int i = 0; i < MAT_NUMBER; i++)
        fitingMat[i] = _linkingMat[i] + _linkedMat[i];
    }
    else if (_linkingMat != NULL)
    {
      for (int i = 0; i < MAT_NUMBER; i++)
        fitingMat[i] = _linkingMat[i];
    }
    else if (_linkedMat != NULL)
    {
      for (int i = 0; i < MAT_NUMBER; i++)
        fitingMat[i] = _linkedMat[i];
    }


    double ellipse_error;
    
    fit_ellipse(fitingMat, ellipse_error, fitres);

    // Some obviously cases: 无法拟合出椭圆，椭圆尺寸大于图像尺寸，圆心在图像外
    if (ellipse_error < 0)
      return false;
    if (fitres.center.x < 0 || fitres.center.x >= iROWS || fitres.center.y<0 || fitres.center.y>iCOLS)
      return false;
    if (fitres.size.height*fitres.size.width > iROWS*iCOLS)
      return false;

    return true;
  }


  void AAMED::fit_ellipse(double * data, double & error, cv::RotatedRect &res)
  {
    double rp[5], _buf_fit_data[36];
    double a1p, a2p, a11p, a22p, C2;
    int dot_num = data[14];
    cv::Mat buf_fit_data(6, 6, CV_64FC1, _buf_fit_data);
    cv::Point2d cen_dot(data[12] / (2 * dot_num), data[13] / (2 * dot_num));//the center of the fitting data.

    _buf_fit_data[0] = data[0] - 2 * cen_dot.x*data[3] + 6 * cen_dot.x*cen_dot.x*data[5] - 3 * dot_num*pow(cen_dot.x, 4);
    _buf_fit_data[1] = data[1] - 3 * cen_dot.x*data[4] + 3 * cen_dot.x*cen_dot.x*data[8] - cen_dot.y*data[3] + cen_dot.x*cen_dot.y * 6 * data[5] - 6 * dot_num*cen_dot.y*pow(cen_dot.x, 3);
    _buf_fit_data[2] = data[2] - cen_dot.y*data[4] + cen_dot.y*cen_dot.y*data[5] - cen_dot.x*data[7] / 2 + cen_dot.x*cen_dot.y*data[8] * 2 + cen_dot.x*cen_dot.x*data[11] - 3 * dot_num*pow(cen_dot.x*cen_dot.y, 2);
    _buf_fit_data[3] = data[3] - 6 * cen_dot.x*data[5] + 4 * dot_num*pow(cen_dot.x, 3);
    _buf_fit_data[4] = data[4] - cen_dot.x*data[8] * 2 - 2 * cen_dot.y*data[5] + 4 * dot_num*cen_dot.x*cen_dot.x*cen_dot.y;
    _buf_fit_data[5] = data[5] - dot_num*cen_dot.x*cen_dot.x;

    _buf_fit_data[1 * 6 + 1] = 4 * _buf_fit_data[2];
    _buf_fit_data[1 * 6 + 2] = data[6] - 1.5 * cen_dot.y*data[7] + 3 * cen_dot.y*cen_dot.y*data[8] - cen_dot.x*data[10] + 6 * cen_dot.x*cen_dot.y*data[11] - 6 * dot_num*cen_dot.x*pow(cen_dot.y, 3);
    _buf_fit_data[1 * 6 + 3] = 2 * _buf_fit_data[4];
    _buf_fit_data[1 * 6 + 4] = data[7] - 4 * cen_dot.y*data[8] - 4 * cen_dot.x*data[11] + 8 * dot_num*cen_dot.x*cen_dot.y*cen_dot.y;
    _buf_fit_data[1 * 6 + 5] = data[8] - 2 * dot_num*cen_dot.x*cen_dot.y;

    _buf_fit_data[2 * 6 + 2] = data[9] - 2 * cen_dot.y*data[10] + 6 * cen_dot.y*cen_dot.y*data[11] - 3 * dot_num*pow(cen_dot.y, 4);
    _buf_fit_data[2 * 6 + 3] = _buf_fit_data[1 * 6 + 4] / 2;
    _buf_fit_data[2 * 6 + 4] = data[10] - 6 * cen_dot.y*data[11] + 4 * dot_num*pow(cen_dot.y, 3);
    _buf_fit_data[2 * 6 + 5] = data[11] - dot_num*cen_dot.y*cen_dot.y;

    _buf_fit_data[3 * 6 + 3] = 4 * _buf_fit_data[5];
    _buf_fit_data[3 * 6 + 4] = 2 * _buf_fit_data[1 * 6 + 5];
    _buf_fit_data[3 * 6 + 5] = 0;

    _buf_fit_data[4 * 6 + 4] = 4 * _buf_fit_data[2 * 6 + 5];
    _buf_fit_data[4 * 6 + 5] = 0;

    _buf_fit_data[5 * 6 + 5] = dot_num;
    for (int i = 0; i < 6; i++)
    {
      for (int j = i + 1; j < 6; j++)
      {
        _buf_fit_data[j * 6 + i] = _buf_fit_data[i * 6 + j];
      }
    }
    double t1, t2;
    
    eigen(buf_fit_data, dls_D, dls_V);

    double *_dls_D = (double*)dls_D.data;
    if (_dls_D[5] > 1e-10)
    {
      for (int i = 0; i < 6; i++)
        dls_V.row(i) = dls_V.row(i) / sqrt(fabs(_dls_D[i]));
      buf_fit_data = dls_V*dls_C*dls_V.t();
      eigen(buf_fit_data, dls_D2, dls_V2);
      //		cout << dls_D2 << '\n' << dls_V2 << endl;
      double *_dls_D2 = (double*)dls_D2.data;
      if (_dls_D2[0] > 0)
      {
        error = PIXEL_SCALE*PIXEL_SCALE / _dls_D2[0] / dot_num;
        dls_X = dls_V2.row(0)*dls_V;
      }
      else
      {
        error = -1;
        return;
      }
    }
    else
      dls_X = dls_V.row(5);
    //cout << dls_X << endl;
    //system("pause");
    double *_dls_X = (double*)dls_X.data;
    double dls_k = _dls_X[0] * _dls_X[2] - _dls_X[1] * _dls_X[1];
    dls_X = dls_X / sqrt(abs(dls_k));



    rp[0] = _dls_X[1] * _dls_X[4] - _dls_X[2] * _dls_X[3];
    rp[1] = _dls_X[1] * _dls_X[3] - _dls_X[0] * _dls_X[4];
    if (fabs(_dls_X[0] - _dls_X[2]) > 1e-10)
      rp[4] = atan(2 * _dls_X[1] / (_dls_X[0] - _dls_X[2])) / 2;
    else
    {
      if (_dls_X[1] > 0)
        rp[4] = CV_PI / 4;
      else
        rp[4] = -CV_PI / 4;
    }
    //至此拟合出来的参数信息是以左上角0,0位置为原点，row为x轴，col为y轴为基准的
    a1p = cos(rp[4])*_dls_X[3] + sin(rp[4])*_dls_X[4];
    a2p = -sin(rp[4])*_dls_X[3] + cos(rp[4])*_dls_X[4];
    a11p = _dls_X[0] + tan(rp[4])*_dls_X[1];
    a22p = _dls_X[2] - tan(rp[4])*_dls_X[1];
    C2 = a1p*a1p / a11p + a2p*a2p / a22p - _dls_X[5];
    double dls_temp1 = C2 / a11p, dls_temp2 = C2 / a22p, dls_temp;
    if (dls_temp1 > 0 && dls_temp2 > 0)
    {
      rp[2] = sqrt(dls_temp1);
      rp[3] = sqrt(dls_temp2);
      if (rp[2] < rp[3])
      {
        if (rp[4] >= 0)
          rp[4] -= CV_PI / 2;
        else
          rp[4] += CV_PI / 2;
        dls_temp = rp[2];
        rp[2] = rp[3];
        rp[3] = dls_temp;
      }
    }
    else
    {
      error = -2;
      return;
    }
    res.center.x = (rp[0] + cen_dot.x)*PIXEL_SCALE;
    res.center.y = (rp[1] + cen_dot.y)*PIXEL_SCALE;
    res.size.width = 2 * rp[2] * PIXEL_SCALE;
    res.size.height = 2 * rp[3] * PIXEL_SCALE;
    res.angle = rp[4] / CV_PI * 180;
  }


  bool AAMED::fast_validation(cv::RotatedRect &res, double *detScore, 
    cv::Mat &imgEdge)
  {
    const float angleRot(res.angle / 180 * CV_PI),
      xyCenter[2] = { res.center.x, res.center.y },
      R(res.size.width / 2), r(res.size.height / 2);// The major-axis and minor-axis.

    // Shape constraint I_0(R,r).  Note: sqrt(2) - 1 ~= 0.414213562373095

    // Shape Index
    if (R*r * 4 < _T_min_minor * _T_min_minor)
      return false;
    if (R*r < 2 * _T_dp / (0.414213562373095)*sqrt(R*R + r * r))
      return false;
    
    const float _ROT_TRANS[4] = { R * cos(angleRot), -r * sin(angleRot), R  * sin(angleRot), r  * cos(angleRot) },
      _ROT_GRAD[4] = { -r * sin(angleRot), -R * cos(angleRot), r*cos(angleRot), -R * sin(angleRot) };

    float m = r / R;

    if (m < 0.2) { detScore = 0; return false; } // \B3\F6\CF\D6\D4\E0\CA\FD\BEݣ\ACʹ\D3\C3\D5ⷽ\B7\A8\BD\E2\BE\F6
    if (m < 0.4) m = 0.4;

    float tmpw, vldBaseData_x, vldBaseData_y, tmpx, tmpy, tmpgx, tmpgy;
    for (int i = 0; i < VALIDATION_NUMBER; i++)
    {
      vldBaseData_x = vldBaseData[i][0];
      vldBaseData_y = vldBaseData[i][1];
      tmpw = m / (m*m*vldBaseData_x * vldBaseData_x + vldBaseData_y * vldBaseData_y);
      tmpx = _ROT_TRANS[0] * vldBaseData_x + _ROT_TRANS[1] * vldBaseData_y + xyCenter[0];
      tmpy = _ROT_TRANS[2] * vldBaseData_x + _ROT_TRANS[3] * vldBaseData_y + xyCenter[1];
      tmpgx = _ROT_GRAD[0] * vldBaseData_x + _ROT_GRAD[1] * vldBaseData_y;
      tmpgy = _ROT_GRAD[2] * vldBaseData_x + _ROT_GRAD[3] * vldBaseData_y;


      sample_x[i] = tmpx;
      sample_y[i] = tmpy;
      grad_x[i] = tmpgx;
      grad_y[i] = tmpgy;
      sample_weight[i] = tmpw;
    }




    float step;
    int RoundEllipseCircum = int((R + r)*CV_PI);
    if (RoundEllipseCircum > 360) RoundEllipseCircum = 360;
    step = 360.0 / RoundEllipseCircum;

    const int vld_num = _T_gradnum;

    int x, y, idxdxy, idxixy, angle_idx;
    int xOffset, yOffset, xReal, yReal, idxdxyReal, count1, count2, length_l_i_2, length_g_i_2;
    float w, sum_w(0), norm_li_gi, E_score(0);
    unsigned char* _boldData = (unsigned char*)imgEdge.data;
    Node_FC *node_temp(NULL), *node_next = NULL, *node_last = NULL;
    cv::Point l_i;
    cv::Point2f g_i;
    float inSw = 0, outSw = 0, inNum = 0, onNum = 0;
    for (int i = 0; i < RoundEllipseCircum; i++)
    {
      angle_idx = round(i*step);
      if (angle_idx >= VALIDATION_NUMBER)
      {
        RoundEllipseCircum = i;
        break;
      }
      w = sample_weight[angle_idx];
      //w = 1;
      sum_w += w;

      x = sample_x[angle_idx], y = sample_y[angle_idx];
      if (OutOfRange(x, y))
      {
        outSw += w;
        continue;
      }
      inSw += w;
      inNum += 1;

      idxdxy = dIDX(x, y), idxixy = iIDX(x, y);

      if (_boldData[idxixy] == 0)
      {
        E_score += w * 0.5;
        continue;
      }
      onNum += 1;
      //\BB\F1ȡ\D5\E6ʵ\B1\DFԵ\B5\E3
      xOffset = _boldData[idxixy] / 10; yOffset = _boldData[idxixy] - 10 * xOffset;
      xOffset = xOffset - 1; yOffset = yOffset - 2;
      xReal = x - xOffset; yReal = y - yOffset;
      idxdxyReal = dIDX(xReal, yReal);
      // \B9\C0\BCƵ\B1ǰ\B5\E3\B5\C4\CCݶ\C8ֵ
      
      node_temp = data + idxdxyReal;
      node_last = node_next = NULL;
      for (count1 = 0; count1 < vld_num; count1++)
      {
        if (node_temp == NULL)
          break;
        node_next = node_temp;
        node_temp = node_temp->nextAddress;
        //				node_next = node_next->nextAddress;
      }
      node_temp = data + idxdxyReal;
      for (count2 = 0; count2 < vld_num; count2++)
      {
        if (node_temp == NULL)
          break;
        node_last = node_temp;
        node_temp = node_temp->lastAddress;
      }
      if (count1 + count2 < vld_num)
        continue;
      // Grad l_i Estimation
      l_i = node_next->Location - node_last->Location;

      length_l_i_2 = l_i.x*l_i.x + l_i.y*l_i.y;
      // Grad g_i Estimation
      g_i.x = grad_x[angle_idx], g_i.y = grad_y[angle_idx];
      length_g_i_2 = g_i.x*g_i.x + g_i.y*g_i.y;

      // |g_i * l_i| / (|g_i|*|l_i|)
      norm_li_gi = abs(l_i.x*g_i.x + l_i.y*g_i.y) / sqrt(length_l_i_2*length_g_i_2);
      //if (norm_li_gi <= 0.707106781186548) // sqrt(2)/2 ~= 0.707106781186548
      //	continue;
      //E_score += w*(norm_li_gi - 0.707106781186548) / (1 - 0.707106781186548);
      if (norm_li_gi > 1) norm_li_gi = 1;
      E_score += w * abs(1 - 2 / CV_PI * acos(norm_li_gi));

      //E_score += w;

    }

    if (outSw > inSw)
      E_score = 0;
    else
    {
      E_score = E_score / inSw * inNum;
    }
    //E_score = E_score / sum_w*RoundEllipseCircum;


    if (E_score > inNum * _T_val)
    {
      *detScore = E_score / inNum;
      return true;
    }
    else
      return false;
  }


// }