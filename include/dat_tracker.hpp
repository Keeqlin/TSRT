#ifndef DAT_TRACKER_HPP_
#define DAT_TRACKER_HPP_
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#define _PI 3.141592653589793
#define _2PI 6.283185307179586


struct dat_cfg {
	bool show_figures = true;
	int  img_scale_target_diagonal = 75;
	double search_win_padding = 2;
	double surr_win_factor = 1.9;
	int color_space = 3; //1rgb 2lab 3hsv 4gray
	int num_bins = 16;
	cv::Mat bin_mapping; //getBinMapping(cfg.num_bins);
	double prob_lut_update_rate = 0.3;
	bool distractor_aware = false;
	std::vector<double> adapt_thresh_prob_bins; // 0:0.05 : 1;
	int motion_estimation_history_size = 3;

	int nms_scale = 1;
	double nms_overlap = 0.9;
	double nms_score_factor = 0.5;
	bool nms_include_center_vote = true;
};

class DAT_TRACKER
{
public:
	DAT_TRACKER(){ cfg = default_parameters_dat(cfg); };
	~DAT_TRACKER(){}

	void tracker_dat_initialize(cv::Mat I, cv::Rect region);
	
	cv::Rect tracker_dat_update(cv::Mat I);

protected:
	void getNMSRects(cv::Mat prob_map, cv::Size obj_sz, double scale,
		double overlap, double score_frac, cv::Mat dist_map, bool include_inner,
		std::vector<cv::Rect> &top_rects, std::vector<double> &top_vote_scores, std::vector<double> &top_dist_scores);

	void getForegroundBackgroundProbs(cv::Mat frame, cv::Rect obj_rect, int num_bins, cv::Mat bin_mapping, cv::Mat &prob_lut, cv::Mat &prob_map);

	void getForegroundBackgroundProbs(cv::Mat frame, cv::Rect obj_rect, int num_bins, cv::Mat &prob_lut);

	cv::Mat getForegroundDistractorProbs(cv::Mat frame, cv::Rect obj_rect, std::vector<cv::Rect> distractors, int num_bins);

	double getAdaptiveThreshold(cv::Mat prob_map, cv::Rect obj_rect_surr);

	cv::Size Scale_estimation(cv::Mat& prob_map, cv::Rect& obj_rect_surr);

	cv::Size region_growing(cv::Mat& tmp_map, cv::Rect& obj_rect_surr, std::queue<cv::Point>& seeds);

	cv::Mat getForegroundProb(cv::Mat frame, cv::Mat prob_lut);

	cv::Mat CalculateHann(cv::Size sz);

	double intersectionOverUnion(cv::Rect target_rect, cv::Rect candidates);

	void getSubwindowMasked(cv::Mat im, cv::Point pos, cv::Size sz, cv::Mat &out, cv::Mat &mask);

	cv::Point getMotionPrediction(std::vector<cv::Point>values, int maxNumFrames);

	cv::Rect pos2rect(const cv::Point& obj_center, const cv::Size& obj_size, const cv::Size& win_size);

	cv::Rect pos2rect(const cv::Point& obj_center, const cv::Size& obj_size);

	cv::Mat getSubwindow(const cv::Mat &frame, cv::Point centerCoor, cv::Size sz);

	dat_cfg default_parameters_dat(dat_cfg cfg);

	void convert_color_space(cv::Mat img);

	std::string type2str(int type);

	inline void generate_prob_map(cv::Mat &prob_map,cv::Mat &Input_img, cv::Mat &prob_lut);

	cv::Point get_Rect_center(cv::Rect& rect);

	cv::Mat prob_visualization(cv::Mat &prob_map);

	private:
		dat_cfg cfg;
		double scale_factor_;
		cv::Mat prob_lut_;
		cv::Mat prob_lut_distractor_;
		cv::Mat prob_lut_masked_;
		double adaptive_threshold_;
		std::vector<cv::Point> target_pos_history_;
		std::vector<cv::Size> target_sz_history_;
};

#endif /* DAT_TRACKER_HPP_ */