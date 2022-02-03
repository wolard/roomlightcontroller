#ifndef PARSER_H
#define PARSER_H
#include <NeoPixelBrightnessBus.h> 
class Parser
{
private:
    const int _COMMLENGTH=1;   
    int _commnum;
    NeoPixelBus<NeoGrbwFeature, NeoSk6812Method> *_strip;
    void execCommand(int cmd);

public:

        Parser(NeoPixelBus<NeoGrbwFeature, NeoSk6812Method> *strip);
        void getMsg(char *msg);
     
  
}; 

#endif