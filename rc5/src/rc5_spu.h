#ifndef __rc5_spu_h__
#define __rc5_spu_h__

void data_shuffle( unsigned int* inputarray, unsigned int* temparray, int size) {
  int offset;
  int i;
  offset = size >> 1;
  unsigned ishift;
  
  memcpy( temparray, inputarray, size * sizeof( unsigned int ) );
  for( i = 0 ; i < size ; i += 8 ) {
    ishift = i>>1;
    inputarray[ishift] = temparray[i+0];
    inputarray[ishift+1] = temparray[i+2];
    inputarray[ishift+2] = temparray[i+4];
    inputarray[ishift+3] = temparray[i+6];

    inputarray[offset+ishift] = temparray[i+1];
    inputarray[offset+ishift+1] = temparray[i+3];
    inputarray[offset+ishift+2] = temparray[i+5];
    inputarray[offset+ishift+3] = temparray[i+7];
  }
}

void data_restore( unsigned int* inputarray, unsigned int* temparray, int size ) {
	int offset;
	int i;
	unsigned ishift;
  	offset = size >> 1;

	memcpy( temparray, inputarray, size * sizeof( unsigned int ) );
	for( i = 0 ; i < size ; i += 8 ) {
	  ishift = i>>1;
	  inputarray[i + 0] = temparray[ishift];
	  inputarray[i + 1] = temparray[offset + ishift];
	  inputarray[i + 2] = temparray[ishift+1];
	  inputarray[i + 3] = temparray[offset + ishift + 1];
	  inputarray[i + 4] = temparray[ishift+2];
	  inputarray[i + 5] = temparray[offset + ishift + 2];
	  inputarray[i + 6] = temparray[ishift+3];
	  inputarray[i + 7] = temparray[offset + ishift + 3];
	}
}

#endif
