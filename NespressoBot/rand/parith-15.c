#include "parith-15.h"

/* POLY erzeugt GF(2^15) = GF(2)[x] */
/* POLY = 1 + x^2 + x^5 + x^8 + x^13 + x^14 + x^15 */
#define POLY 0xe125 
#define DEGREE 15

/* ROOT ist ein zyklischer Erzeuger von GF(2^15)^* */
/* ROOT = x^9 + x^13 */
#define ROOT 0x2200 

typedef unsigned short poly;

static poly seed = ROOT;

poly pprod  (poly, poly);

#define MASK(b) \
	(((poly) 1) << (b))
	
/**
 * Arithmetik in GF(2)[x] / p(x)*GF(2)[x]
 * Berechnet c = a*b mod p
 */
poly pprod (poly a, poly b)
{
	const poly mask = MASK (DEGREE);
	poly c = 0;

	do
	{
		// Ist Bit i in b gesetzt?
		if (b & 1)
			c ^= a;		// dann c = c + a * x^i
            
		a <<= 1;		// a = a*x
		if (a & mask)	// a = a mod p
			a ^= POLY;

		b >>= 1;
	} while (b);

	return c;
}

// Liefert eine Pseudo-Zufallszahl x
// mit 1 <= x <= 2^DEGREE-1 = 32767
unsigned short prandom (void)
{
    // seed *= ROOT 
	return (unsigned short) (seed = pprod (seed, ROOT));
}

void set_seed (unsigned short aseed)
{
	// aseed ist poly vom Grad <= DEGREE-1
	aseed &= MASK(DEGREE) -1;
	
	if (0 == aseed)
		aseed = 1;
	
	seed = aseed;
}
