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

~include <algorithm>
~include <cstring>

~include "CompressedVectorNodeImpl.h"
~include "Decoder.h"
~include "FloatNodeImpl.h"
~include "ImageFileImpl.h"
~include "IntegerNodeImpl.h"
~include "ScaledIntegerNodeImpl.h"
~include "SourceDestBufferImpl.h"

using namespace e57;

std::shared_ptr<Decoder> Decoder::DecoderFactory( unsigned bytestreamNumber, //!!! name ok?
                                                  const CompressedVectorNodeImpl *cVector,
                                                  std::vector<SourceDestBuffer> &dbufs, const ustring & /*codecPath*/ )
{
   //!!! verify single dbuf

   /// Get node we are going to decode from the CompressedVector's prototype
   NodeImplSharedPtr prototype = cVector->getPrototype();
   ustring path = dbufs.at( 0 ).pathName();
   NodeImplSharedPtr decodeNode = prototype->get( path );

~ifdef E57_MAX_VERBOSE
   std::cout << "Node to decode:" << std::endl; //???
   decodeNode->dump( 2 );
~endif

   uint64_t maxRecordCount = cVector->childCount();

   switch ( decodeNode->type() )
   {
      case E57_INTEGER:
      {
         std::shared_ptr<IntegerNodeImpl> ini =
            std::static_pointer_cast<IntegerNodeImpl>( decodeNode ); // downcast to correct type

         /// Get pointer to parent ImageFileImpl, to call bitsNeeded()
         ImageFileImplSharedPtr imf( decodeNode->destImageFile_ ); //??? should be function for this,
                                                                   // imf->parentFile()
                                                                   //--> ImageFile?

         unsigned bitsPerRecord = imf->bitsNeeded( ini->minimum(), ini->maximum() );

         //!!! need to pick smarter channel buffer sizes, here and elsewhere
         /// Constuct Integer decoder with appropriate register size, based on
         /// number of bits stored.
         if ( bitsPerRecord == 0 )
         {
            std::shared_ptr<Decoder> decoder( new ConstantIntegerDecoder( false, bytestreamNumber, dbufs.at( 0 ),
                                                                          ini->minimum(), 1.0, 0.0, maxRecordCount ) );
            return decoder;
         }

         if ( bitsPerRecord <= 8 )
         {
            std::shared_ptr<Decoder> decoder( new BitpackIntegerDecoder<uint8_t>(
               false, bytestreamNumber, dbufs.at( 0 ), ini->minimum(), ini->maximum(), 1.0, 0.0, maxRecordCount ) );
            return decoder;
         }

         if ( bitsPerRecord <= 16 )
         {
            std::shared_ptr<Decoder> decoder( new BitpackIntegerDecoder<uint16_t>(
               false, bytestreamNumber, dbufs.at( 0 ), ini->minimum(), ini->maximum(), 1.0, 0.0, maxRecordCount ) );
            return decoder;
         }

         if ( bitsPerRecord <= 32 )
         {
            std::shared_ptr<Decoder> decoder( new BitpackIntegerDecoder<uint32_t>(
               false, bytestreamNumber, dbufs.at( 0 ), ini->minimum(), ini->maximum(), 1.0, 0.0, maxRecordCount ) );
            return decoder;
         }

         std::shared_ptr<Decoder> decoder( new BitpackIntegerDecoder<uint64_t>(
            false, bytestreamNumber, dbufs.at( 0 ), ini->minimum(), ini->maximum(), 1.0, 0.0, maxRecordCount ) );
         return decoder;
      }

      case E57_SCALED_INTEGER:
      {
         std::shared_ptr<ScaledIntegerNodeImpl> sini =
            std::static_pointer_cast<ScaledIntegerNodeImpl>( decodeNode ); // downcast to correct type

         /// Get pointer to parent ImageFileImpl, to call bitsNeeded()
         ImageFileImplSharedPtr imf( decodeNode->destImageFile_ ); //??? should be function for this,
                                                                   // imf->parentFile()
                                                                   //--> ImageFile?

         unsigned bitsPerRecord = imf->bitsNeeded( sini->minimum(), sini->maximum() );

         //!!! need to pick smarter channel buffer sizes, here and elsewhere
         /// Constuct ScaledInteger dencoder with appropriate register size,
         /// based on number of bits stored.
         if ( bitsPerRecord == 0 )
         {
            std::shared_ptr<Decoder> decoder( new ConstantIntegerDecoder( true, bytestreamNumber, dbufs.at( 0 ),
                                                                          sini->minimum(), sini->scale(),
                                                                          sini->offset(), maxRecordCount ) );
            return decoder;
         }

         if ( bitsPerRecord <= 8 )
         {
            std::shared_ptr<Decoder> decoder(
               new BitpackIntegerDecoder<uint8_t>( true, bytestreamNumber, dbufs.at( 0 ), sini->minimum(),
                                                   sini->maximum(), sini->scale(), sini->offset(), maxRecordCount ) );
            return decoder;
         }

         if ( bitsPerRecord <= 16 )
         {
            std::shared_ptr<Decoder> decoder(
               new BitpackIntegerDecoder<uint16_t>( true, bytestreamNumber, dbufs.at( 0 ), sini->minimum(),
                                                    sini->maximum(), sini->scale(), sini->offset(), maxRecordCount ) );
            return decoder;
         }

         if ( bitsPerRecord <= 32 )
         {
            std::shared_ptr<Decoder> decoder(
               new BitpackIntegerDecoder<uint32_t>( true, bytestreamNumber, dbufs.at( 0 ), sini->minimum(),
                                                    sini->maximum(), sini->scale(), sini->offset(), maxRecordCount ) );
            return decoder;
         }

         std::shared_ptr<Decoder> decoder(
            new BitpackIntegerDecoder<uint64_t>( true, bytestreamNumber, dbufs.at( 0 ), sini->minimum(),
                                                 sini->maximum(), sini->scale(), sini->offset(), maxRecordCount ) );
         return decoder;
      }

      case E57_FLOAT:
      {
         std::shared_ptr<FloatNodeImpl> fni =
            std::static_pointer_cast<FloatNodeImpl>( decodeNode ); // downcast to correct type

         std::shared_ptr<Decoder> decoder(
            new BitpackFloatDecoder( bytestreamNumber, dbufs.at( 0 ), fni->precision(), maxRecordCount ) );
         return decoder;
      }

      case E57_STRING:
      {
         std::shared_ptr<Decoder> decoder(
            new BitpackStringDecoder( bytestreamNumber, dbufs.at( 0 ), maxRecordCount ) );

         return decoder;
      }

      default:
      {
         throw E57_EXCEPTION2( E57_ERROR_BAD_PROTOTYPE, "nodeType=" + toString( decodeNode->type() ) );
      }
   }
}

Decoder::Decoder( unsigned bytestreamNumber ) : bytestreamNumber_( bytestreamNumber )
{
}

BitpackDecoder::BitpackDecoder( unsigned bytestreamNumber, SourceDestBuffer &dbuf, unsigned alignmentSize,
                                uint64_t maxRecordCount ) :
   Decoder( bytestreamNumber ),
   maxRecordCount_( maxRecordCount ), destBuffer_( dbuf.impl() ),
   inBuffer_( 1024 ), //!!! need to pick smarter channel buffer sizes
   inBufferAlignmentSize_( alignmentSize ), bitsPerWord_( 8 * alignmentSize ), bytesPerWord_( alignmentSize )
{
}

void BitpackDecoder::destBufferSetNew( std::vector<SourceDestBuffer> &dbufs )
{
   if ( dbufs.size() != 1 )
   {
      throw E57_EXCEPTION2( E57_ERROR_INTERNAL, "dbufsSize=" + toString( dbufs.size() ) );
   }

   destBuffer_ = dbufs.at( 0 ).impl();
}

size_t BitpackDecoder::inputProcess( const char *source, const size_t availableByteCount )
{
~ifdef E57_MAX_VERBOSE
   std::cout << "BitpackDecoder::inputprocess() called, source=" << ( source ? source : "none" )
             << " availableByteCount=" << availableByteCount << std::endl;
~endif
   size_t bytesUnsaved = availableByteCount;
   size_t bitsEaten = 0;
   do
   {
      size_t byteCount = std::min( bytesUnsaved, inBuffer_.size() - static_cast<size_t>( inBufferEndByte_ ) );

      /// Copy input bytes from caller, if any
      if ( ( byteCount > 0 ) && ( source != nullptr ) )
      {
         memcpy( &inBuffer_[inBufferEndByte_], source, byteCount );

         /// Advance tail pointer.
         inBufferEndByte_ += byteCount;

         /// Update amount available from caller
         bytesUnsaved -= byteCount;
         source += byteCount;
      }
~ifdef E57_MAX_VERBOSE
      {
         unsigned i;
         unsigned firstByte = inBufferFirstBit_ / 8;
         for ( i = 0; i < byteCount && i < 20; i++ )
         {
            std::cout << "  inBuffer[" << firstByte + i << "]=" << (unsigned)(unsigned char)( inBuffer_[firstByte + i] )
                      << std::endl;
         }
         if ( i < byteCount )
         {
            std::cout << "  " << byteCount - i << "source bytes unprinted..." << std::endl;
         }
      }
~endif

      /// ??? fix doc for new bit interface
      /// Now that we have input stored in an aligned buffer, call derived class
      /// to try to eat some Note that end of filled buffer may not be at a
      /// natural boundary. The subclass may transfer this partial word in a
      /// full word transfer, but it must be carefull to only use the defined
      /// bits. inBuffer_ is a multiple of largest word size, so this full word
      /// transfer off the end will always be in defined memory.

      size_t firstWord = inBufferFirstBit_ / bitsPerWord_;
      size_t firstNaturalBit = firstWord * bitsPerWord_;
      size_t endBit = inBufferEndByte_ * 8;
~ifdef E57_MAX_VERBOSE
      std::cout << "  feeding aligned decoder " << endBit - inBufferFirstBit_ << " bits." << std::endl;
~endif
      bitsEaten = inputProcessAligned( &inBuffer_[firstWord * bytesPerWord_], inBufferFirstBit_ - firstNaturalBit,
                                       endBit - firstNaturalBit );
~ifdef E57_MAX_VERBOSE
      std::cout << "  bitsEaten=" << bitsEaten << " firstWord=" << firstWord << " firstNaturalBit=" << firstNaturalBit
                << " endBit=" << endBit << std::endl;
~endif
~ifdef E57_DEBUG
      if ( bitsEaten > endBit - inBufferFirstBit_ )
      {
         throw E57_EXCEPTION2( E57_ERROR_INTERNAL, "bitsEaten=" + toString( bitsEaten ) +
                                                      " endBit=" + toString( endBit ) +
                                                      " inBufferFirstBit=" + toString( inBufferFirstBit_ ) );
      }
~endif
      inBufferFirstBit_ += bitsEaten;

      /// Shift uneaten data to beginning of inBuffer_, keep on natural word
      /// boundaries.
      inBufferShiftDown();

      /// If the lower level processing didn't eat anything on this iteration,
      /// stop looping and tell caller how much we ate or stored.
   } while ( bytesUnsaved > 0 && bitsEaten > 0 );

   /// Return the number of bytes we ate/saved.
   return ( availableByteCount - bytesUnsaved );
}

void BitpackDecoder::stateReset()
{
   inBufferFirstBit_ = 0;
   inBufferEndByte_ = 0;
}

void BitpackDecoder::inBufferShiftDown()
{
   /// Move uneaten data down to beginning of inBuffer_.
   /// Keep on natural boundaries.
   /// Moves all of word that contains inBufferFirstBit.
   size_t firstWord = inBufferFirstBit_ / bitsPerWord_;
   size_t firstNaturalByte = firstWord * bytesPerWord_;
~ifdef E57_DEBUG
   if ( firstNaturalByte > inBufferEndByte_ )
   {
      throw E57_EXCEPTION2( E57_ERROR_INTERNAL, "firstNaturalByte=" + toString( firstNaturalByte ) +
                                                   " inBufferEndByte=" + toString( inBufferEndByte_ ) );
   }
~endif
   size_t byteCount = inBufferEndByte_ - firstNaturalByte;
   if ( byteCount > 0 )
   {
      memmove( &inBuffer_[0], &inBuffer_[firstNaturalByte],
               byteCount ); /// Overlapping regions ok with memmove().
   }

   /// Update indexes
   inBufferEndByte_ = byteCount;
   inBufferFirstBit_ = inBufferFirstBit_ % bitsPerWord_;
}

~ifdef E57_DEBUG
void BitpackDecoder::dump( int indent, std::ostream &os )
{
   os << space( indent ) << "bytestreamNumber:         " << bytestreamNumber_ << std::endl;
   os << space( indent ) << "currentRecordIndex:       " << currentRecordIndex_ << std::endl;
   os << space( indent ) << "maxRecordCount:           " << maxRecordCount_ << std::endl;
   os << space( indent ) << "destBuffer:" << std::endl;
   destBuffer_->dump( indent + 4, os );
   os << space( indent ) << "inBufferFirstBit:        " << inBufferFirstBit_ << std::endl;
   os << space( indent ) << "inBufferEndByte:         " << inBufferEndByte_ << std::endl;
   os << space( indent ) << "inBufferAlignmentSize:   " << inBufferAlignmentSize_ << std::endl;
   os << space( indent ) << "bitsPerWord:             " << bitsPerWord_ << std::endl;
   os << space( indent ) << "bytesPerWord:            " << bytesPerWord_ << std::endl;
   os << space( indent ) << "inBuffer:" << std::endl;
   unsigned i;
   for ( i = 0; i < inBuffer_.size() && i < 20; i++ )
   {
      os << space( indent + 4 ) << "inBuffer[" << i
         << "]: " << static_cast<unsigned>( static_cast<unsigned char>( inBuffer_.at( i ) ) ) << std::endl;
   }
   if ( i < inBuffer_.size() )
   {
      os << space( indent + 4 ) << inBuffer_.size() - i << " more unprinted..." << std::endl;
   }
}
~endif

//================================================================

BitpackFloatDecoder::BitpackFloatDecoder( unsigned bytestreamNumber, SourceDestBuffer &dbuf, FloatPrecision precision,
                                          uint64_t maxRecordCount ) :
   BitpackDecoder( bytestreamNumber, dbuf, ( precision == E57_SINGLE ) ? sizeof( float ) : sizeof( double ),
                   maxRecordCount ),
   precision_( precision )
{
}

size_t BitpackFloatDecoder::inputProcessAligned( const char *inbuf, const size_t firstBit, const size_t endBit )
{
~ifdef E57_MAX_VERBOSE
   std::cout << "BitpackFloatDecoder::inputProcessAligned() called, inbuf=" << inbuf << " firstBit=" << firstBit
             << " endBit=" << endBit << std::endl;
~endif
   /// Read from inbuf, decode, store in destBuffer
   /// Repeat until have filled destBuffer, or completed all records

   size_t n = destBuffer_->capacity() - destBuffer_->nextIndex();

   size_t typeSize = ( precision_ == E57_SINGLE ) ? sizeof( float ) : sizeof( double );

~ifdef E57_DEBUG
~if 0 // I know no way to do this portably <rs>
   // Deactivate for now until a better solution is found.
   /// Verify that inbuf is naturally aligned to correct boundary (4 or 8 bytes).  Base class should be doing this for us.
   if (reinterpret_cast<unsigned>(inbuf) % typeSize) {
      throw E57_EXCEPTION2(E57_ERROR_INTERNAL,
                           "inbuf=" + toString(reinterpret_cast<unsigned>(inbuf))
                           + " typeSize=" + toString(typeSize));
   }
~endif
   /// Verify first bit is zero
   if ( firstBit != 0 )
   {
      throw E57_EXCEPTION2( E57_ERROR_INTERNAL, "firstBit=" + toString( firstBit ) );
   }
~endif

   /// Calc how many whole records worth of data we have in inbuf
   size_t maxInputRecords = ( endBit - firstBit ) / ( 8 * typeSize );

   /// Can't process more records than we have input data for.
   if ( n > maxInputRecords )
   {
      n = maxInputRecords;
   }

   // Can't process more than defined in input file
   if ( n > maxRecordCount_ - currentRecordIndex_ )
   {
      n = static_cast<unsigned>( maxRecordCount_ - currentRecordIndex_ );
   }

~ifdef E57_MAX_VERBOSE
   std::cout << "  n:" << n << std::endl; //???
~endif

   if ( precision_ == E57_SINGLE )
   {
      /// Form the starting address for first data location in inBuffer
      auto inp = reinterpret_cast<const float *>( inbuf );

      /// Copy floats from inbuf to destBuffer_
      for ( unsigned i = 0; i < n; i++ )
      {
         float value = *inp;

~ifdef E57_MAX_VERBOSE
         std::cout << "  got float value=" << value << std::endl;
~endif
         destBuffer_->setNextFloat( value );
         inp++;
      }
   }
   else
   { /// E57_DOUBLE precision
      /// Form the starting address for first data location in inBuffer
      auto inp = reinterpret_cast<const double *>( inbuf );

      /// Copy doubles from inbuf to destBuffer_
      for ( unsigned i = 0; i < n; i++ )
      {
         double value = *inp;

~ifdef E57_MAX_VERBOSE
         std::cout << "  got double value=" << value << std::endl;
~endif
         destBuffer_->setNextDouble( value );
         inp++;
      }
   }

   /// Update counts of records processed
   currentRecordIndex_ += n;

   /// Returned number of bits processed  (always a multiple of alignment size).
   return ( n * 8 * typeSize );
}

~ifdef E57_DEBUG
void BitpackFloatDecoder::dump( int indent, std::ostream &os )
{
   BitpackDecoder::dump( indent, os );
   if ( precision_ == E57_SINGLE )
   {
      os << space( indent ) << "precision:                E57_SINGLE" << std::endl;
   }
   else
   {
      os << space( indent ) << "precision:                E57_DOUBLE" << std::endl;
   }
}
~endif

//================================================================

BitpackStringDecoder::BitpackStringDecoder( unsigned bytestreamNumber, SourceDestBuffer &dbuf,
                                            uint64_t maxRecordCount ) :
   BitpackDecoder( bytestreamNumber, dbuf, sizeof( char ), maxRecordCount )
{
}

size_t BitpackStringDecoder::inputProcessAligned( const char *inbuf, const size_t firstBit, const size_t endBit )
{
~ifdef E57_MAX_VERBOSE
   std::cout << "BitpackStringDecoder::inputProcessAligned() called, inbuf=" << inbuf << " firstBit=" << firstBit
             << " endBit=" << endBit << std::endl;
~endif
   /// Read from inbuf, decode, store in destBuffer
   /// Repeat until have filled destBuffer, or completed all records

~ifdef E57_DEBUG
   /// Verify first bit is zero (always byte-aligned)
   if ( firstBit != 0 )
   {
      throw E57_EXCEPTION2( E57_ERROR_INTERNAL, "firstBit=" + toString( firstBit ) );
   }
~endif

   /// Converts start/end bits to whole bytes
   size_t nBytesAvailable = ( endBit - firstBit ) >> 3;
   size_t nBytesRead = 0;

   /// Loop until we've finished all the records, or ran out of input currently
   /// available
   while ( currentRecordIndex_ < maxRecordCount_ && nBytesRead < nBytesAvailable )
   {
~ifdef E57_MAX_VERBOSE
      std::cout << "read string loop1: readingPrefix=" << readingPrefix_ << " prefixLength=" << prefixLength_
                << " nBytesPrefixRead=" << nBytesPrefixRead_ << " nBytesStringRead=" << nBytesStringRead_ << std::endl;
~endif
      if ( readingPrefix_ )
      {
         /// Try to read more prefix bytes
         while ( nBytesRead < nBytesAvailable && ( nBytesPrefixRead_ == 0 || nBytesPrefixRead_ < prefixLength_ ) )
         {
            /// If first byte of prefix, test the least significant bit to see
            /// how long prefix is
            if ( nBytesPrefixRead_ == 0 )
            {
               if ( *inbuf & 0x01 )
               {
                  prefixLength_ = 8; // 8 byte prefix, length upto 2^63-1
               }
               else
               {
                  prefixLength_ = 1; // 1 byte prefix, length upto 2^7-1
               }
            }

            /// Accumulate prefix bytes
            prefixBytes_[nBytesPrefixRead_] = *inbuf++;
            nBytesPrefixRead_++;
            nBytesRead++;
         }

~ifdef E57_MAX_VERBOSE
         std::cout << "read string loop2: readingPrefix=" << readingPrefix_ << " prefixLength=" << prefixLength_
                   << " nBytesPrefixRead=" << nBytesPrefixRead_ << " nBytesStringRead=" << nBytesStringRead_
                   << std::endl;
~endif
         /// If got all of prefix, convert to length and get ready to read
         /// string
         if ( nBytesPrefixRead_ > 0 && nBytesPrefixRead_ == prefixLength_ )
         {
            if ( prefixLength_ == 1 )
            {
               /// Single byte prefix, extract length from b7-b1.
               /// Removing the least significant bit (which says this is a
               /// short prefix).
               stringLength_ = static_cast<uint64_t>( prefixBytes_[0] >> 1 );
            }
            else
            {
               /// Eight byte prefix, extract length from b63-b1. Little endian
               /// ordering. Removing the least significant bit (which says this
               /// is a long prefix).
               stringLength_ = ( static_cast<uint64_t>( prefixBytes_[0] ) >> 1 ) +
                               ( static_cast<uint64_t>( prefixBytes_[1] ) << ( 1 * 8 - 1 ) ) +
                               ( static_cast<uint64_t>( prefixBytes_[2] ) << ( 2 * 8 - 1 ) ) +
                               ( static_cast<uint64_t>( prefixBytes_[3] ) << ( 3 * 8 - 1 ) ) +
                               ( static_cast<uint64_t>( prefixBytes_[4] ) << ( 4 * 8 - 1 ) ) +
                               ( static_cast<uint64_t>( prefixBytes_[5] ) << ( 5 * 8 - 1 ) ) +
                               ( static_cast<uint64_t>( prefixBytes_[6] ) << ( 6 * 8 - 1 ) ) +
                               ( static_cast<uint64_t>( prefixBytes_[7] ) << ( 7 * 8 - 1 ) );
            }
            /// Get ready to read string contents
            readingPrefix_ = false;
            prefixLength_ = 1;
            memset( prefixBytes_, 0, sizeof( prefixBytes_ ) );
            nBytesPrefixRead_ = 0;
            currentString_ = "";
            nBytesStringRead_ = 0;
         }
~ifdef E57_MAX_VERBOSE
         std::cout << "read string loop3: readingPrefix=" << readingPrefix_ << " prefixLength=" << prefixLength_
                   << " nBytesPrefixRead=" << nBytesPrefixRead_ << " nBytesStringRead=" << nBytesStringRead_
                   << std::endl;
~endif
      }

      /// If currently reading string contents, keep doing it until have
      /// complete string
      if ( !readingPrefix_ )
      {
         /// Calc how many bytes we need to complete current string
         uint64_t nBytesNeeded = stringLength_ - nBytesStringRead_;

         /// Can process the smaller of unread or needed bytes
         size_t nBytesProcess = nBytesAvailable - nBytesRead;
         if ( nBytesNeeded < static_cast<uint64_t>( nBytesProcess ) )
         {
            nBytesProcess = static_cast<unsigned>( nBytesNeeded );
         }

         /// Append to current string and update counts
         currentString_ += ustring( inbuf, nBytesProcess );
         inbuf += nBytesProcess;
         nBytesRead += nBytesProcess;
         nBytesStringRead_ += nBytesProcess;

         /// Check if completed reading the string contents
         if ( nBytesStringRead_ == stringLength_ )
         {
            /// Save accumulated string to dest buffer
            destBuffer_->setNextString( currentString_ );
            currentRecordIndex_++;

            /// Get ready to read next prefix
            readingPrefix_ = true;
            prefixLength_ = 1;
            memset( prefixBytes_, 0, sizeof( prefixBytes_ ) );
            nBytesPrefixRead_ = 0;
            stringLength_ = 0;
            currentString_ = "";
            nBytesStringRead_ = 0;
         }
      }
   }

   /// Returned number of bits processed  (always a multiple of alignment size).
   return ( nBytesRead * 8 );
}

~ifdef E57_DEBUG
void BitpackStringDecoder::dump( int indent, std::ostream &os )
{
   BitpackDecoder::dump( indent, os );
   os << space( indent ) << "readingPrefix:      " << readingPrefix_ << std::endl;
   os << space( indent ) << "prefixLength:       " << prefixLength_ << std::endl;
   os << space( indent ) << "prefixBytes[8]:     " << static_cast<unsigned>( prefixBytes_[0] ) << " "
      << static_cast<unsigned>( prefixBytes_[1] ) << " " << static_cast<unsigned>( prefixBytes_[2] ) << " "
      << static_cast<unsigned>( prefixBytes_[3] ) << " " << static_cast<unsigned>( prefixBytes_[4] ) << " "
      << static_cast<unsigned>( prefixBytes_[5] ) << " " << static_cast<unsigned>( prefixBytes_[6] ) << " "
      << static_cast<unsigned>( prefixBytes_[7] ) << std::endl;
   os << space( indent ) << "nBytesPrefixRead:   " << nBytesPrefixRead_ << std::endl;
   os << space( indent ) << "stringLength:       " << stringLength_ << std::endl;
   os << space( indent )
      << "currentString:      "
         ""
      << currentString_
      << ""
         ""
      << std::endl;
   os << space( indent ) << "nBytesStringRead:   " << nBytesStringRead_ << std::endl;
}
~endif

//================================================================

template <typename RegisterT>
BitpackIntegerDecoder<RegisterT>::BitpackIntegerDecoder( bool isScaledInteger, unsigned bytestreamNumber,
                                                         SourceDestBuffer &dbuf, int64_t minimum, int64_t maximum,
                                                         double scale, double offset, uint64_t maxRecordCount ) :
   BitpackDecoder( bytestreamNumber, dbuf, sizeof( RegisterT ), maxRecordCount ),
   isScaledInteger_( isScaledInteger ), minimum_( minimum ), maximum_( maximum ), scale_( scale ), offset_( offset )
{
   /// Get pointer to parent ImageFileImpl
   ImageFileImplSharedPtr imf( dbuf.impl()->destImageFile() ); //??? should be function for this,
                                                               // imf->parentFile()  --> ImageFile?

   bitsPerRecord_ = imf->bitsNeeded( minimum_, maximum_ );
   destBitMask_ = ( bitsPerRecord_ == 64 ) ? ~0 : static_cast<RegisterT>( 1ULL << bitsPerRecord_ ) - 1;
}

template <typename RegisterT>
size_t BitpackIntegerDecoder<RegisterT>::inputProcessAligned( const char *inbuf, const size_t firstBit,
                                                              const size_t endBit )
{
~ifdef E57_MAX_VERBOSE
   std::cout << "BitpackIntegerDecoder::inputProcessAligned() called, inbuf=" << (void *)( inbuf )
             << " firstBit=" << firstBit << " endBit=" << endBit << std::endl;
~endif

   /// Read from inbuf, decode, store in destBuffer
   /// Repeat until have filled destBuffer, or completed all records

~ifdef E57_DEBUG
~if 0 // I know now way to do this portably
   // Deactivate for now until a better solution is found.
   /// Verify that inbuf is naturally aligned to RegisterT boundary (1, 2, 4,or 8 bytes).  Base class is doing this for us.
   if ((reinterpret_cast<unsigned>(inbuf)) % sizeof(RegisterT))
      throw E57_EXCEPTION2(E57_ERROR_INTERNAL, "inbuf=" + toString(reinterpret_cast<unsigned>(inbuf)));
~endif
   /// Verfiy first bit is in first word
   if ( firstBit >= 8 * sizeof( RegisterT ) )
   {
      throw E57_EXCEPTION2( E57_ERROR_INTERNAL, "firstBit=" + toString( firstBit ) );
   }
~endif

   size_t destRecords = destBuffer_->capacity() - destBuffer_->nextIndex();

   /// Precalculate exact number of full records that are in inbuf
   /// We can handle the case where don't have a full word at end of inbuf, but
   /// all the bits of the record are there;
   size_t bitCount = endBit - firstBit;
   size_t maxInputRecords = bitCount / bitsPerRecord_;

   /// Number of transfers is the smaller of what was requested and what is
   /// available in input.
   size_t recordCount = std::min( destRecords, maxInputRecords );

   // Can't process more than defined in input file
   if ( static_cast<uint64_t>( recordCount ) > maxRecordCount_ - currentRecordIndex_ )
   {
      recordCount = static_cast<unsigned>( maxRecordCount_ - currentRecordIndex_ );
   }

~ifdef E57_MAX_VERBOSE
   std::cout << "  recordCount=" << recordCount << std::endl;
~endif

   auto inp = reinterpret_cast<const RegisterT *>( inbuf );
   unsigned wordPosition = 0; /// The index in inbuf of the word we are currently working on.

   ///  For example on little endian machine:
   ///  Assume: registerT=uint32_t, bitOffset=20, destBitMask=0x00007fff (for a
   ///  15 bit value). inp[wordPosition]                    LLLLLLLL LLLLXXXX
   ///  XXXXXXXX XXXXXXXX   Note LSB of value is at bit20 inp(wordPosition+1]
   ///  XXXXXXXX XXXXXXXX XXXXXXXX XXXXXHHH H=high bits of value,
   ///  X=uninteresting bits low = inp[i] >> bitOffset            00000000
   ///  00000000 0000LLLL LLLLLLLL   L=low bits of value, X=uninteresting bits
   ///  high = inp[i+1] << (32-bitOffset)    XXXXXXXX XXXXXXXX XHHH0000 00000000
   ///  w = high | low XXXXXXXX XXXXXXXX XHHHLLLL LLLLLLLL destBitmask 00000000
   ///  00000000 01111111 11111111 w & mask                             00000000
   ///  00000000 0HHHLLLL LLLLLLLL

   size_t bitOffset = firstBit;

   for ( size_t i = 0; i < recordCount; i++ )
   {
      /// Get lower word (contains at least the LSbit of the value),
      RegisterT low = inp[wordPosition];

~ifdef E57_MAX_VERBOSE
      std::cout << "  bitOffset: " << bitOffset << std::endl;
      std::cout << "  low: " << binaryString( low ) << std::endl;
~endif

      RegisterT w;
      if ( bitOffset > 0 )
      {
         /// Get upper word (may or may not contain interesting bits),
         RegisterT high = inp[wordPosition + 1];

~ifdef E57_MAX_VERBOSE
         std::cout << "  high:" << binaryString( high ) << std::endl;
~endif

         /// Shift high to just above the lower bits, shift low LSBit to bit0,
         /// OR together. Note shifts are logical (not arithmetic) because using
         /// unsigned variables.
         w = ( high << ( 8 * sizeof( RegisterT ) - bitOffset ) ) | ( low >> bitOffset );
      }
      else
      {
         /// The left shift (used above) is not defined if shift is >= size of
         /// word
         w = low;
      }

~ifdef E57_MAX_VERBOSE
      std::cout << "  w:   " << binaryString( w ) << std::endl;
~endif

      /// Mask off uninteresting bits
      w &= destBitMask_;

      /// Add minimum_ to value to get back what writer originally sent
      int64_t value = minimum_ + static_cast<uint64_t>( w );

~ifdef E57_MAX_VERBOSE
      std::cout << "  Storing value=" << value << std::endl;
~endif

      /// The parameter isScaledInteger_ determines which version of
      /// setNextInt64 gets called
      if ( isScaledInteger_ )
      {
         destBuffer_->setNextInt64( value, scale_, offset_ );
      }
      else
      {
         destBuffer_->setNextInt64( value );
      }

      /// Store the result in next avaiable position in the user's dest buffer

      /// Calc next bit alignment and which word it starts in
      bitOffset += bitsPerRecord_;
      if ( bitOffset >= 8 * sizeof( RegisterT ) )
      {
         bitOffset -= 8 * sizeof( RegisterT );
         wordPosition++;
      }
~ifdef E57_MAX_VERBOSE
      std::cout << "  Processed " << i + 1 << " records, wordPosition=" << wordPosition << " decoder:" << std::endl;
      dump( 4 );
~endif
   }

   /// Update counts of records processed
   currentRecordIndex_ += recordCount;

   /// Return number of bits processed.
   return ( recordCount * bitsPerRecord_ );
}

~ifdef E57_DEBUG
template <typename RegisterT> void BitpackIntegerDecoder<RegisterT>::dump( int indent, std::ostream &os )
{
   BitpackDecoder::dump( indent, os );
   os << space( indent ) << "isScaledInteger:  " << isScaledInteger_ << std::endl;
   os << space( indent ) << "minimum:          " << minimum_ << std::endl;
   os << space( indent ) << "maximum:          " << maximum_ << std::endl;
   os << space( indent ) << "scale:            " << scale_ << std::endl;
   os << space( indent ) << "offset:           " << offset_ << std::endl;
   os << space( indent ) << "bitsPerRecord:    " << bitsPerRecord_ << std::endl;
   os << space( indent ) << "destBitMask:      " << binaryString( destBitMask_ ) << " = " << hexString( destBitMask_ )
      << std::endl;
}
~endif

//================================================================

ConstantIntegerDecoder::ConstantIntegerDecoder( bool isScaledInteger, unsigned bytestreamNumber, SourceDestBuffer &dbuf,
                                                int64_t minimum, double scale, double offset,
                                                uint64_t maxRecordCount ) :
   Decoder( bytestreamNumber ),
   maxRecordCount_( maxRecordCount ), destBuffer_( dbuf.impl() ), isScaledInteger_( isScaledInteger ),
   minimum_( minimum ), scale_( scale ), offset_( offset )
{
}

void ConstantIntegerDecoder::destBufferSetNew( std::vector<SourceDestBuffer> &dbufs )
{
   if ( dbufs.size() != 1 )
   {
      throw E57_EXCEPTION2( E57_ERROR_INTERNAL, "dbufsSize=" + toString( dbufs.size() ) );
   }

   destBuffer_ = dbufs.at( 0 ).impl();
}

size_t ConstantIntegerDecoder::inputProcess( const char *source, const size_t availableByteCount )
{
   (void)source; (void)availableByteCount;
~ifdef E57_MAX_VERBOSE
   std::cout << "ConstantIntegerDecoder::inputprocess() called, source=" << (void *)( source )
             << " availableByteCount=" << availableByteCount << std::endl;
~endif

   /// We don't need any input bytes to produce output, so ignore source and
   /// availableByteCount.

   /// Fill dest buffer unless get to maxRecordCount
   size_t count = destBuffer_->capacity() - destBuffer_->nextIndex();
   uint64_t remainingRecordCount = maxRecordCount_ - currentRecordIndex_;
   if ( static_cast<uint64_t>( count ) > remainingRecordCount )
   {
      count = static_cast<unsigned>( remainingRecordCount );
   }

   if ( isScaledInteger_ )
   {
      for ( size_t i = 0; i < count; i++ )
      {
         destBuffer_->setNextInt64( minimum_, scale_, offset_ );
      }
   }
   else
   {
      for ( size_t i = 0; i < count; i++ )
      {
         destBuffer_->setNextInt64( minimum_ );
      }
   }
   currentRecordIndex_ += count;
   return ( count );
}

void ConstantIntegerDecoder::stateReset()
{
}

~ifdef E57_DEBUG
void ConstantIntegerDecoder::dump( int indent, std::ostream &os )
{
   os << space( indent ) << "bytestreamNumber:   " << bytestreamNumber_ << std::endl;
   os << space( indent ) << "currentRecordIndex: " << currentRecordIndex_ << std::endl;
   os << space( indent ) << "maxRecordCount:     " << maxRecordCount_ << std::endl;
   os << space( indent ) << "isScaledInteger:    " << isScaledInteger_ << std::endl;
   os << space( indent ) << "minimum:            " << minimum_ << std::endl;
   os << space( indent ) << "scale:              " << scale_ << std::endl;
   os << space( indent ) << "offset:             " << offset_ << std::endl;
   os << space( indent ) << "destBuffer:" << std::endl;
   destBuffer_->dump( indent + 4, os );
}
~endif
