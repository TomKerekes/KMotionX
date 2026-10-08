#include "KMotionTChar.h"

// driver.h


extern KMTCHAR  _interpreter_linetext[];
extern KMTCHAR  _interpreter_blocktext[];


int read_tool_file(      /* ARGUMENT VALUES             */
 const KMTCHAR * tool_file,       /* name of tool file           */
 setup_pointer settings); /* pointer to machine settings */

int save_tool_file(const KMTCHAR* File);  // save tool file with occasional backup
int save_tool_file_0(const KMTCHAR* File);  // save tool file


int read_setup_file(     /* ARGUMENT VALUES             */
 const KMTCHAR * setup_file,      /* name of setup file          */
 setup_pointer settings); /* pointer to machine settings */

