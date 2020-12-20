#ifndef __rc5_h__
#define __rc5_h__

#define P 0xb7e15163
#define Q 0x9e3779b9

#define w 32


//#define DATASIZE 2097152         /* DATASIZE words*/
#define w        32             /* word size in bits                 */
#define wb       4              /*word size in bytes*/
#define r        12             /* number of rounds                  */  
#define b        16           /* number of bytes in key            */
#define c        4           /* number  words in key = ceil(8*b/w)*/
#define t        26             /* size of table S = 2*(r+1) words   */
#define NO_SPU 4


#define VECTOR_SIZE (DATASIZE/NO_SPU)/* must be multiple of 1024, spu program is optimized under this assumption(e.g. loop unrolling) */

typedef struct _control_block {
  unsigned int addrinputdata;
  unsigned int addroutputdata;
  unsigned int addrexpandedtable;
  unsigned int DATASIZE;
  unsigned char pad[112];
} control_block;


void rc5_Encrypt(unsigned int* input_data, unsigned int * encrypted_data, unsigned int DATASIZE);
void rc5_Decrypt(unsigned int* encrypted_data, unsigned int * decrypted_data, unsigned int DATASIZE);
int rc5_Init(unsigned int DATASIZE);
int rc5_Check(unsigned int* input_data,unsigned int * decrypted_data, unsigned int DATASIZE);

#endif

