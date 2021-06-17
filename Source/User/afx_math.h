#ifndef __afx_math_h
#define __afx_math_h

#define sq(x)                       ((x) * (x))
#define constrain(v, lo, hi)		(((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))
#define constrainInt(v, lo, hi)	    (((int)(v) < (int)(lo)) ? (int)(lo) : (((int)(v) > (int)(hi)) ? (int)(hi) : (int)(v)))
#define constrainFloat(v, lo, hi)   (((float)(v) < (float)(lo)) ? (float)(lo) : (((float)(v) > (float)(hi)) ? (float)(hi) : (float)(v)))

#endif /* __afx_math_h */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************** (C) COPYRIGHT AEE Technology ******END OF FILE*****/
