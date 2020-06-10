// A C/C++ program for splitting a string 
// using strtok() 
#include <stdio.h> 
#include <string.h> 
#include <stdlib.h>

int main() 
{ 
FILE *stream;
char c[100]; 
int i, tok[4],addr;
//        stream = fopen("/home/peter/ge_fits.dat", "r"); 
//   for(i=0;i<4;i++){
//       fgets(c, 100, stream);
//       printf("Data from the file:\n%s", c);
//       }
	addr=0;
	sprintf(c,"172.16.0.150");
	// Returns first token 
	char* token = strtok(c, "."); 
	printf("%s\n", token); 

	tok[0]=atoi(token);
	
	// Keep printing tokens while one of the 
	// delimiters present in str[]. 
	for(i=0;i<3;i++) {  
		token = strtok(NULL, "."); 
		printf("%s\n", token); 
		tok[i+1]=atoi(token);
	} 
	addr=tok[0]*16777216+tok[1]*65536+tok[2]*256+tok[3];
	
	printf("Addr = %ul\n",addr);
	return 0; 
} 
