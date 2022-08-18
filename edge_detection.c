#include "edge_detection.h"

typedef struct
{
	bool o_aux;
}edge_detection_t;

edge_detection_t s_ed_obj[ED_OBJ_NUM];


bool edge_detection(uint8_t b_id, bool o_catch)
{
	bool o_retval = false;
	
	if (o_catch)
	{
		if (!s_ed_obj[b_id].o_aux)
		{
			s_ed_obj[b_id].o_aux = true;
			o_retval = true;
		}
	}
	else
	{
		s_ed_obj[b_id].o_aux = false;
	}
	return o_retval;
}
