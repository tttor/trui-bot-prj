#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class BadmintonMap {
 public:
 	BadmintonMap(double res): kScale_(1.0/res) {
 		build();
 	}

 	~BadmintonMap() {
 		delete map_;
 	}

	void save(const std::string& path) {
		// TODO @tttor: also create the config file: map.yaml
 		cv::imwrite(path, *map_);
 	}

 private:
 	/**
	 	All scaled units are in pixels.
	 	All un-scaled units are in meters.
 	*/
 	void build() {
 		// Dims of the contest field
 		const double contest_field_width = scale(8.500);
 		const double contest_field_length = scale(16.400);
 		const double line_width = scale(0.040);

 		// Addition areas for operator zone
 		const double op_zone = scale(1.000);

 		map_ = new cv::Mat(contest_field_length+(op_zone*2), contest_field_width+(op_zone*2), CV_8UC1, cv::Scalar(255));

 		//
 		cv::Rect contest_field(cv::Point(op_zone,op_zone), cv::Point(op_zone+contest_field_width, op_zone+contest_field_length));
 		cv::rectangle(*map_, contest_field, cv::Scalar(0), line_width, CV_AA);

 		//
 		const double main_field_width = scale(6.180);
 		const double main_field_length = scale(13.480);

 		cv::Rect main_field(cv::Point(op_zone+scale(1.160), op_zone+scale(1.460)), cv::Point(op_zone+scale(1.160)+main_field_width, op_zone+scale(1.460)+main_field_length));
 		cv::rectangle(*map_, main_field, cv::Scalar(150), line_width, CV_AA);

 		// The middle separation line
 		cv::line(*map_, cv::Point(op_zone,op_zone+contest_field_length/2), cv::Point(op_zone+contest_field_width,op_zone+contest_field_length/2), cv::Scalar(0), line_width, CV_AA); 
 	}

 	double scale(const double& measure) {
 		return measure * kScale_;
 	}

 	const double kScale_; // one-pixel : actual-measure
 	cv::Mat* map_;
};

int main() {
	using namespace std;

	const double kRes = 0.01;// in meter per pixel
	BadmintonMap map(kRes);
	map.save("/home/tor/trui/controller/src/rbmt_map/map/badminton.png");

	return 0;
}
	