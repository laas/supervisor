#ifndef SCAN_DOMAIN_H
#define SCAN_DOMAIN_H

#include "plan_elaboration/virtual_domain.h"

using namespace std;

class ScanDomain: public VirtualDomain
{
public:
    ScanDomain();
    ~ScanDomain() {};
    virtual vector<toaster_msgs::Fact> computeSpecificFacts(vector<toaster_msgs::Fact> facts);

private:
    string areaScan_;
    vector<toaster_msgs::Fact> computeScanArea(vector<toaster_msgs::Fact> facts);
};

#endif // SCAN_DOMAIN_H
