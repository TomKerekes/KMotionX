// driver.h


extern TCHAR  _interpreter_linetext[];
extern TCHAR  _interpreter_blocktext[];


int read_tool_file(      /* ARGUMENT VALUES             */
 TCHAR * tool_file,       /* name of tool file           */
 setup_pointer settings); /* pointer to machine settings */

int save_tool_file(const TCHAR* File);  // save tool file with occasional backup
int save_tool_file_0(const TCHAR* File);  // save tool file


int read_setup_file(     /* ARGUMENT VALUES             */
 TCHAR * setup_file,      /* name of setup file          */
 setup_pointer settings); /* pointer to machine settings */

