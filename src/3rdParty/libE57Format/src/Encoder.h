/*
 * Original work Copyright 2009 - 2010 Kevin Ackley (kackley@gwi.net)
 * Modified work Copyright 2018 - 2020 Andy Maloney <asmaloney@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer,
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include "Common.h"

namespace e57
{
   class Encoder
   {
   public:
      static std::shared_ptr<Encoder> EncoderFactory( unsigned bytestreamNumber,
                                                      std::shared_ptr<CompressedVectorNodeImpl> cVector,
                                                      std::vector<SourceDestBuffer> &sbuf, ustring &codecPath );

      virtual ~Encoder() = default;

      virtual uint64_t processRecords( size_t recordCount ) = 0;
      virtual unsigned sourceBufferNextIndex() = 0;
      virtual uint64_t currentRecordIndex() = 0;
      virtual float bitsPerRecord() = 0;
      virtual bool registerFlushToOutput() = 0;

      /// number of bytes that can be read
      virtual size_t outputAvailable() const = 0;
      /// get data from encoder
      virtual void outputRead( char *dest, const size_t byteCount ) = 0;
      virtual void outputClear() = 0;

      virtual void sourceBufferSetNew( std::vector<SourceDestBuffer> &sbufs ) = 0;
      virtual size_t outputGetMaxSize() = 0;
      virtual void outputSetMaxSize( unsigned byteCount ) = 0;

      unsigned bytestreamNumber() const
      {
         return bytestreamNumber_;
      }

#ifdef E57_DEBUG
      virtual void dump( int indent = 0, std::ostream &os = std::cout ) const;
#endif
   protected:
      Encoder( unsigned bytestreamNumber );

      unsigned bytestreamNumber_;
   };

   class BitpackEncoder : public Encoder
   {
   public:
      uint64_t processRecords( size_t recordCount ) override = 0;
      unsigned sourceBufferNextIndex() override;
      uint64_t currentRecordIndex() override;
      float bitsPerRecord() override = 0;
      bool registerFlushToOutput() override = 0;

      size_t outputAvailable() const override; /// number of bytes that can be read
      void outputRead( char *dest,
                       const size_t byteCount ) override; /// get data from encoder
      void outputClear() override;

      void sourceBufferSetNew( std::vector<SourceDestBuffer> &sbufs ) override;
      size_t outputGetMaxSize() override;
      void outputSetMaxSize( unsigned byteCount ) override;

#ifdef E57_DEBUG
      void dump( int indent = 0, std::ostream &os = std::cout ) const override;
#endif
   protected:
      BitpackEncoder( unsigned bytestreamNumber, SourceDestBuffer &sbuf, unsigned outputMaxSize,
                      unsigned alignmentSize );

      void outBufferShiftDown();

      std::shared_ptr<SourceDestBufferImpl> sourceBuffer_;

      std::vector<char> outBuffer_;
      size_t outBufferFirst_;
      size_t outBufferEnd_;
      size_t outBufferAlignmentSize_;

      uint64_t currentRecordIndex_;
   };

   class BitpackFloatEncoder : public BitpackEncoder
   {
   public:
      BitpackFloatEncoder( unsigned bytestreamNumber, SourceDestBuffer &sbuf, unsigned outputMaxSize,
                           FloatPrecision precision );

      uint64_t processRecords( size_t recordCount ) override;
      bool registerFlushToOutput() override;
      float bitsPerRecord() override;

#ifdef E57_DEBUG
      void dump( int indent = 0, std::ostream &os = std::cout ) const override;
#endif
   protected:
      FloatPrecision precision_;
   };

   class BitpackStringEncoder : public BitpackEncoder
   {
   public:
      BitpackStringEncoder( unsigned bytestreamNumber, SourceDestBuffer &sbuf, unsigned outputMaxSize );

      uint64_t processRecords( size_t recordCount ) override;
      bool registerFlushToOutput() override;
      float bitsPerRecord() override;

#ifdef E57_DEBUG
      void dump( int indent = 0, std::ostream &os = std::cout ) const override;
#endif
   protected:
      uint64_t totalBytesProcessed_;
      bool isStringActive_;
      bool prefixComplete_;
      ustring currentString_;
      size_t currentCharPosition_;
   };

   template <typename RegisterT> class BitpackIntegerEncoder : public BitpackEncoder
   {
   public:
      BitpackIntegerEncoder( bool isScaledInteger, unsigned bytestreamNumber, SourceDestBuffer &sbuf,
                             unsigned outputMaxSize, int64_t minimum, int64_t maximum, double scale, double offset );

      uint64_t processRecords( size_t recordCount ) override;
      bool registerFlushToOutput() override;
      float bitsPerRecord() override;

#ifdef E57_DEBUG
      void dump( int indent = 0, std::ostream &os = std::cout ) const override;
#endif
   protected:
      bool isScaledInteger_;
      int64_t minimum_;
      int64_t maximum_;
      double scale_;
      double offset_;
      unsigned bitsPerRecord_;
      uint64_t sourceBitMask_;
      unsigned registerBitsUsed_;
      RegisterT register_;
   };

   class ConstantIntegerEncoder : public Encoder
   {
   public:
      ConstantIntegerEncoder( unsigned bytestreamNumber, SourceDestBuffer &sbuf, int64_t minimum );
      uint64_t processRecords( size_t recordCount ) override;
      unsigned sourceBufferNextIndex() override;
      uint64_t currentRecordIndex() override;
      float bitsPerRecord() override;
      bool registerFlushToOutput() override;

      size_t outputAvailable() const override; /// number of bytes that can be read
      void outputRead( char *dest,
                       const size_t byteCount ) override; /// get data from encoder
      void outputClear() override;

      void sourceBufferSetNew( std::vector<SourceDestBuffer> &sbufs ) override;
      size_t outputGetMaxSize() override;
      void outputSetMaxSize( unsigned byteCount ) override;

#ifdef E57_DEBUG
      void dump( int indent = 0, std::ostream &os = std::cout ) const override;
#endif
   protected:
      std::shared_ptr<SourceDestBufferImpl> sourceBuffer_;
      uint64_t currentRecordIndex_;
      int64_t minimum_;
   };
}
