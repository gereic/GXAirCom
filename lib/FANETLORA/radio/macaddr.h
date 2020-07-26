/*
 * macaddr.h
 *
 *  Created on: Sep 29, 2018
 *      Author: sid
 */

#ifndef FANET_RADIO_MACADDR_H_
#define FANET_RADIO_MACADDR_H_


/*
 * 0, 0 == Broadcast
 */

class MacAddr
{
public:
	int manufacturer;
	int id;

	MacAddr(int manufacturer_addr, int id_addr): manufacturer(manufacturer_addr), id(id_addr) {};
	MacAddr() : manufacturer(0), id(0) {};									//broadcast address
	MacAddr(const MacAddr &ma) : manufacturer(ma.manufacturer), id(ma.id) {};

	inline bool operator == (const MacAddr& rhs) const { return ((id == rhs.id) && (manufacturer == rhs.manufacturer));};
	inline bool operator != (const MacAddr& rhs) const { return ((id != rhs.id) || (manufacturer != rhs.manufacturer));};
};


#endif /* FANET_RADIO_MACADDR_H_ */
