#include "widget.h"

#include <QApplication>
#include "imgprodcons.h"
#include<thread>
#include "common.h"
using namespace std;

int main(int argc, char *argv[])
{

    ImgProdCons imgProdCons;
    imgProdCons.init();
    ourthread *produce ;
    ourthread *consum;

    produce = new ourthread(&imgProdCons,1);
    consum = new ourthread(&imgProdCons,2);

    produce->start();
    consum->start();

    produce->wait();
    consum->wait();
    return 0;


}