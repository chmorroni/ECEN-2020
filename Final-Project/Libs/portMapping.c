#include "portMapping.h"

void mapPin(port portToMap, pin pinToMap, uint8_t mapping) {
	uint8_t bit = 1 << pinToMap;
	PMAP_REGISTER_Type * pNMap = NULL;
	switch (portToMap) {
	case PORT_2:
		P2->SEL0 |= bit;
		P2->SEL1 &= ~bit;
		pNMap = P2MAP;
		break;
	case PORT_3:
		P3->SEL0 |= bit;
		P3->SEL1 &= ~bit;
		pNMap = P3MAP;
		break;
	case PORT_7:
		P7->SEL0 |= bit;
		P7->SEL1 &= ~bit;
		pNMap = P7MAP;
		break;
	}
	PMAP->KEYID = PMAP_KEYID_VAL;                // Unlock PMAP controller
	PMAP->CTL |= PMAP_CTL_PRECFG;                // Allow it to be reconfigured later
	pNMap->PMAP_REGISTER[pinToMap / 2] &= ~((pinToMap % 2) ? 0xFF00 : 0x00FF);
 	pNMap->PMAP_REGISTER[pinToMap / 2] |= mapping << ((pinToMap % 2) * 8);
	PMAP->KEYID = 0;                             // Lock PMAP controller
}
