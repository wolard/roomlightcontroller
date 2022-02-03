#include "Parser.h"

// Date constructor
Parser::Parser(NeoPixelBus<NeoGrbwFeature, NeoSk6812Method> *strip)
{
   _strip=strip;
   _strip->Begin();

}
void Parser::getMsg(char* msg)

{

    if((msg[0]>='0')&&(msg[0]<='9'))
    {
        _commnum=msg[0];
        this->execCommand(_commnum);
Serial.println(msg[0]);
    }
    
    else return ;

}
void Parser::execCommand(int cmd)
{
switch (cmd)
{
case 48:  //sammuta kaikki
    RgbwColor color(0,0,0,0);
    for (int i=0;i<450;i++)
    {
    _strip->SetPixelColor(i,color);
    }
    _strip->Show();
    break;

}

}


