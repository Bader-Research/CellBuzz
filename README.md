# CellBuzz

The Sony-Toshiba-IBM Cell Broadband Engine is a heterogeneous
multicore architecture that consists of a traditional microprocessor
(PPE) with eight SIMD co-processing units (SPEs) integrated
on-chip. This repository contains Cell Broadband Engine Processor
optimized libraries and software, developed by David A. Bader and his
research group, for a number of libraries for FFT, MPEG, compression,
and encryption. This project also includes patches to accelerate R on
the Cell Broadband Engine processor.

# Software modules

- **fft**: Fast Fourier Transform (FFT) is an efficient algorithm that
is used for computing Discrete Fourier Transform. FFT is of primary
importance and a fundamental kernel in many computationally intensive
scientific applications such as computer tomography, data filtering
and fluid dynamics. Another important application area of FFTs is in
spectral analysis of speech, sonar, radar, seismic and vibration
detection. FFTs are also used in digital filtering, signal
decomposition, and in solution of partial differential equations. The
performance of these applications rely heavily on the availability of
a fast routine for Fourier Transforms. There are several algorithmic
variants of the FFTs that have been well studied for parallel
processors and vector architectures. In our design we utilize the
naive Cooley-Tukey radix-2 Decimate in Frequency algorithm. The
algorithm runs in logN stages and each stage requires O(N)
computation, where N is the input size.
Reference:

-- "[FFTC: Fastest Fourier Transform for the IBM Cell Broadband
   Engine](https://davidbader.net/publication/2007-ba/)," David
   A. Bader and Virat Agarwal, The 14th Annual IEEE International
   Conference on High Performance Computing (HiPC 2007), S. Aluru et
   al., (eds.), Springer-Verlag LNCS 4873, 172-184, Goa, India,
   December 18-21, 2007.

-- "[Computing Discrete Transforms on the Cell Broadband
   Engine](https://davidbader.net/publication/2009-bak/)," David
   A. Bader, Virat Agarwal, and Seunghwa Kang, Parallel Computing,
   35(3):119-137, 2009.

- **jpeg2000**: JPEG2000 is the latest still image coding standard
from the JPEG committee, which adopts new algorithms such as embedded
block coding with optimized truncation (EBCOT) and discrete wavelet
transform (DWT). These algorithms enable superior coding performance
over JPEG and support various new features at the cost of the
increased computational complexity. The Sony-Toshiba-IBM cell
broadband engine (or the Cell/B.E.) is a heterogeneous multicore
architecture with SIMD accelerators. In this work, we optimize the
computationally intensive algorithmic kernels of JPEG2000 for the
Cell/B.E. and also introduce a novel data decomposition scheme to
achieve high performance with low programming complexity. We compare
the Cell/B.E.‘s performance to the performance of the Intel Pentium IV
3.2 GHz processor. The Cell/B.E. demonstrates 3.2 times higher
performance for lossless encoding and 2.7 times higher performance for
lossy encoding. For the DWT, the Cell/B.E. outperforms the Pentium IV
processor by 9.1 times for the lossless case and 15 times for the
lossy case. We also provide the experimental results on one IBM QS20
blade with two Cell/B.E. chips and the performance comparison with the
existing JPEG2000 encoder for the Cell/B.E.
Reference:

-- "[Optimizing JPEG2000 Still Image Encoding on the Cell Broadband
   Engine](https://davidbader.net/publication/2008-kb/)," Seunghwa
   Kang and David A. Bader, 37th International Conference on Parallel
   Processing (ICPP), Portland, OR, September 8-12, 2008.

- **minigzip**: Minigzip for Sony-Toshiba-IBM Cell Broadband Engine
(or Cell processor) is a file compression/decompression utility
similar to gzip (<http://www.gzip.org>). Minigzip, provided here is
programmed using zlib library optimized for Cell Processor. Zlib, a
data compression/decompression library, was originally written by
Jean-loup Gailly and Mark Adler (<http://www.zlib.net>). While zlib is
heavily optimized for sequential execution environment, additional
effort is required to exploit Cell processor's unique architecture.

- **mpeg2**: While the Cell/B.E. processor is designed with multimedia
    applications in mind, there are currently no open-source,
    optimized implementations of such applications available. In this
    paper, we present the design and implementation behind the
    creation of an optimized MPEG-2 software decoder for this unique
    parallel architecture, and demonstrate its performance through an
    experimental study. This is the first parallelization of an MPEG-2
    decoder for a commodity heterogeneous multicore processor such as
    the IBM Cell/B.E. While Drake et al. have recently parallelized
    MPEG-2 using Streamlt for a streaming architecture, our algorithm
    is quite different and is the first to address the new challenges
    related to the optimization and tuning of a multicore algorithm
    with DMA transfers and local store memory. Our design and
    efficient implementation target the architectural features
    provided by the heterogeneous multicore processor. We give an
    experimental study on Sony PlayStation 3 and IBM QS20 dual-Cell
    Blade platforms. For instance, using 16 SPEs on the IBM QS20, our
    decoder runs 3.088 times faster than a 3.2 GHz Intel Xeon and
    achieves a speedup of over 10.545 compared with a PPE-only
    implementation. Our source code is freely- available through
    SourceForge under the CellBuzz project.
    Reference:

-- [High Performance MPEG-2 Software Decoder on the Cell Broadband
   Engine](https://davidbader.net/publication/2008-b-pa/)," David
   A. Bader and Sulabh Patel, 22nd IEEE International Parallel and
   Distributed Processing Symposium (IPDPS), Miami, FL, April 14-18,
   2008.

- **rc5**: RC5 is a block cipher that was originally designed by
    Ronald Rivest
    [paper](http://theory.lcs.mit.edu/~cis/pubs/rivest/rc5.ps). We
    have implemented and optimized the RC5 encryption/decryption
    library for the Sony-Toshiba-IBM Cell Broadband Engine. RC5 is
    widely used in message encryption, digital signatures, stream
    encryption, internet e-commerce, file encryption, electronic cash,
    etc. This library has been implemented by Virat Agarwal and David
    A. Bader.

- **zlib**: The Sony–Toshiba–IBM Cell Broadband Engine (Cell/B.E.) is
    a heterogeneous multicore architecture that consists of a
    traditional microprocessor (PPE) with eight SIMD co-processing
    units (SPEs) integrated on-chip. While the Cell/B.E. processor is
    architected for multimedia applications with regular processing
    requirements, we are interested in its performance on problems
    with non-uniform memory access patterns. In this article, we
    present two case studies to illustrate the design and
    implementation of parallel combinatorial algorithms on Cell/B.E.:
    we discuss list ranking, a fundamental kernel for graph problems,
    and zlib, a data compression and decompression library. List
    ranking is a particularly challenging problem to parallelize on
    current cache-based and distributed memory architectures due to
    its low computational intensity and irregular memory access
    patterns. To tolerate memory latency on the Cell/B.E. processor,
    we decompose work into several independent tasks and coordinate
    computation using the novel idea of Software-Managed threads
    (SM-Threads). We apply this generic SPE work-partitioning
    technique to efficiently implement list ranking, and demonstrate
    substantial speedup in comparison to traditional cache-based
    microprocessors. For instance, on a 3.2 GHz IBM QS20
    Cell/B.E. blade, for a random linked list of 1 million nodes, we
    achieve an overall speedup of 8.34 over a PPE-only
    implementation. Our second case study, zlib, is a data
    compression/decompression library that is extensively used in both
    scientific as well as general purpose computing. The core kernels
    in the zlib library are the LZ77 longest subsequence matching
    algorithm and Huffman data encoding. We design efficient parallel
    algorithms for these combinatorial kernels, and exploit
    concurrency at multiple levels on the Cell/B.E. processor. We also
    present a Cell/B.E. optimized implementation of gzip, a popular
    file-compression application based on the zlib library. For our
    Cell/B.E. implementation of gzip, we achieve an average speedup of
    2.9 in compression over current workstations.  
Reference: 

-- "[High Performance Combinatorial Algorithm Design on the Cell
   Broadband Engine
   Processor](https://davidbader.net/publication/2007-bamk/),"
   D.A. Bader, V. Agarwal, K. Madduri, and S. Kang, Parallel
   Computing, 33(10-11):720-740, 2007.




