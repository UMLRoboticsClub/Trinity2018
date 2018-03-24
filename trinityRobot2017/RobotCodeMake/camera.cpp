#include "camera.h"

const int KEYPT_THRESH = 400;
const int BLACK = 40;
const int WHITE = 255;

void illsq ( vector<KeyPoint> pts, double &a, double &b, double c, double d)
  int i;
  double ybar;

  if ( pts.size() == 1 )
  {
    b = pts[0].pt.x;
    return;
  }

  ybar = 0.0;
  double n = 0;
  for ( i = 0; i < pts.size(); i++ )
  {	
	ybar = ybar + pts[i].pt.x * (1 /  (abs(c * (double)pts[i].pt.x + d - (double)pts[i].pt.y) + 100));
	n += (1 /  (abs(c * (double)pts[i].pt.x + d - (double)pts[i].pt.y) + 100));	
  }
  
  ybar = ybar / n;

  b = ybar;

  return;
}
void llsq ( vector<KeyPoint> pts, double &a, double &b)

{
  double bot;
  int i;
  double top;
  double xbar;
  double ybar;

  if ( pts.size() == 1 )
  {
    a = 0.0;
    b = pts[0].pt.y;
    return;
  }

  xbar = 0.0;
  ybar = 0.0;
  for ( i = 0; i < pts.size(); i++ )
  {
	
	    xbar = xbar + pts[i].pt.x;
	    ybar = ybar + pts[i].pt.y;
	
  }
  xbar = xbar / ( double ) pts.size();
  ybar = ybar / ( double ) pts.size();

  top = 0.0;
  bot = 0.0;
  for ( i = 0; i < pts.size(); i++ )
  {

	    top = top + ( pts[i].pt.x - xbar ) * ( pts[i].pt.y - ybar );
	    bot = bot + ( pts[i].pt.x - xbar ) * ( pts[i].pt.x - xbar );
	
  }
  a = top / bot;

  b = ybar- a * xbar;
  return;
}

void Camera::Camera(std::string targetFile) {
	imgo = imread(targetFile, 1);
	threshold(imgo, img, BLACK, WHITE, 0);
	akaze = AKAZE::create();
}
void Camera::poll() {
       stream1.read(img2o);	       
       threshold(img2o, img2, BLACK, WHITE, 0);	    
       akaze->detectAndCompute(img2o, noArray(), kpts2, desc2, false);
       BFMatcher matcher(NORM_HAMMING);
       vector< vector<DMatch> > nn_matches;
       vector<KeyPoint> matched1, matched2;
       matcher.knnMatch(desc1, desc2, nn_matches, 2);
       for(size_t i = 0; i < nn_matches.size(); i++) {
	       DMatch first = nn_matches[i][0];
	       matched1.push_back(kpts1[first.queryIdx]);
	       matched2.push_back(kpts2[first.trainIdx]);	                                                   
       }

       double a, b;
       if(kpts2.size() > KEYPT_THRESH) {
		llsq(kpts2, a, b);
		rotation = a;
		illsq(kpts2, a, b,a, b);
		offset = b;
		return true;
       }
       return false;
}

double Camera::getOffsetX() {
	return offset;
}

double Camera::getRotation() {
	return rotation;
}
