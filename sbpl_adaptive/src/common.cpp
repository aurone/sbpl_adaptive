#include <sbpl_adaptive/common.h>

#include <stdio.h>

#include <sbpl/headers.h>

#define SBPL_ADAPTIVE_PROFILING 0

namespace adim {

StopWatch_t::StopWatch_t()
{
    start_t = sbpl::clock::now();
}

double StopWatch_t::getElapsedSeconds()
{
    return sbpl::to_seconds(sbpl::clock::now() - start_t);
}

void StopWatch_t::reset()
{
    start_t = sbpl::clock::now();
}

void StopWatch_t::print(const std::string& prefix)
{
#if SBPL_ADAPTIVE_PROFILING
    double t_s = getElapsedSeconds();
    int h = (int) floor(t_s / 3600);
    int m = (int) floor((t_s - (3600*h)) / 60);
    int s = (int) floor(t_s - (3600*h) - (60*m));
    double remainder = t_s - (3600*h) - (60*m) - s;
    printf("[StopWatch] %s %d:%02d:%02d.%05d\n", prefix.c_str(), h, m, s, (int)(100000*remainder));
#endif
}

void pause()
{
    printf("Enter to continue...");
    char inp;
    do {
        inp = getchar();
    } while (inp != '\n');
}

bool prompt()
{
    printf("[y/n]?");
    char inp;
    do {
        inp = getchar();
    } while(inp == '\n' || inp == '\r'); //skip enter and carriage return
    if (inp == 'y' || inp == 'Y') {
        return true;
    }
    return false;
}

} // namespace adim
