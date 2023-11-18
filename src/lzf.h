#ifndef _LZF_H
#define	_LZF_H

#include <cerrno>
#include <climits>
#include <cstddef>
#include <algorithm>

// https://github.com/PointCloudLibrary/pcl/blob/master/io/src/lzf.cpp

///////////////////////////////////////////////////////////////////////////////////////////
unsigned int lzfDecompress(const void *const in_data, unsigned int in_len, void *out_data, unsigned int out_len)
{
  auto const *ip = static_cast<const unsigned char *> (in_data);
  auto       *op = static_cast<unsigned char *> (out_data);
  unsigned char const *const in_end = ip + in_len;
  unsigned char       *const out_end = op + out_len;

  do
  {
    unsigned int ctrl = *ip++;

    // Literal run
    if (ctrl < (1 << 5))
    {
      ctrl++;

      if (op + ctrl > out_end)
      {
        errno = E2BIG;
        return (0);
      }

      // Check for overflow
      if (ip + ctrl > in_end)
      {
        errno = EINVAL;
        return (0);
      }
      for (unsigned ctrl_c = ctrl; ctrl_c; --ctrl_c)
        *op++ = *ip++;
    }
    // Back reference
    else
    {
      unsigned int len = ctrl >> 5;

      unsigned char *ref = op - ((ctrl & 0x1f) << 8) - 1;

      // Check for overflow
      if (ip >= in_end)
      {
        errno = EINVAL;
        return (0);
      }
      if (len == 7)
      {
        len += *ip++;
        // Check for overflow
        if (ip >= in_end)
        {
          errno = EINVAL;
          return (0);
        }
      }
      ref -= *ip++;

      if (op + len + 2 > out_end)
      {
        errno = E2BIG;
        return (0);
      }

      if (ref < static_cast<unsigned char *> (out_data))
      {
        errno = EINVAL;
        return (0);
      }

      if (len > 9)
      {
        len += 2;

        if (op >= ref + len)
        {
          // Disjunct
          std::copy(ref, ref + len, op);
          op += len;
        }
        else
        {
          // Overlapping, use byte by byte copying
          do
            *op++ = *ref++;
          while (--len);
        }
      }
      else
        for (unsigned len_c = len + 2 /* case 0 iterates twice */; len_c; --len_c)
          *op++ = *ref++;
    }
  } while (ip < in_end);

  return (static_cast<unsigned int> (op - static_cast<unsigned char*> (out_data)));
}
#endif	/* _LZF_H */
