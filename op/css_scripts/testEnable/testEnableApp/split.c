// A C/C++ program for splitting a string 
// using strtok() 
#include <stdio.h> 
#include <string.h> 

int main() 
{ 
FILE *stream;
char c[100]; 
int i;
        stream = fopen("/home/peter/ge_fits.dat", "r"); 
   for(i=0;i<4;i++){
       fgets(c, 100, stream);
       printf("Data from the file:\n%s", c);
       }

	// Returns first token 
	char* token = strtok(c, " "); 
	
	// Keep printing tokens while one of the 
	// delimiters present in str[]. 
	while (token != NULL) { 
		printf("%s\n", token); 
		token = strtok(NULL, " "); 
	} 

	return 0; 
} 
