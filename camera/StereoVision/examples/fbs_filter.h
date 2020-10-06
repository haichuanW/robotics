#ifndef __FBS_FILTER_H__
#define __FBS_FILTER_H__
#include <opencv2/opencv.hpp>

namespace cv {
namespace ximgproc {

/** @brief Interface for implementations of Fast Bilateral Solver.

For more details about this solver see @cite BarronPoole2016 .
*/
class CV_EXPORTS_W FastBilateralSolverFilter : public Algorithm {
   public:
    /** @brief Apply smoothing operation to the source image.

    @param src source image for filtering with unsigned 8-bit or signed 16-bit or
    floating-point 32-bit depth and up to 3 channels.

    @param confidence confidence image with unsigned 8-bit or floating-point
    32-bit confidence and 1 channel.

    @param dst destination image.

    @note Confidence images with CV_8U depth are expected to in [0, 255] and
    CV_32F in [0, 1] range.
    */
    CV_WRAP virtual void filter(InputArray src, InputArray confidence, OutputArray dst) = 0;
};

CV_EXPORTS_W Ptr<FastBilateralSolverFilter> createFastBilateralSolverFilter(
    InputArray guide, double sigma_spatial, double sigma_luma, double sigma_chroma,
    double lambda = 128.0, int num_iter = 25, double max_tol = 1e-5);

CV_EXPORTS_W void fastBilateralSolverFilter(InputArray guide, InputArray src, InputArray confidence,
                                            OutputArray dst, double sigma_spatial = 8,
                                            double sigma_luma = 8, double sigma_chroma = 8,
                                            double lambda = 128.0, int num_iter = 25,
                                            double max_tol = 1e-5);

}  // namespace ximgproc
}  // namespace cv

#endif  //__FBS_FILTER_H__