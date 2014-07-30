#ifndef __CH_NTB_EEDURO_LATCH_HPP_
#define __CH_NTB_EEDURO_LATCH_HPP_

namespace eeduro
{
	class Latch
	{
	public:
		Latch(bool& ref) : state(false), trigger(true), ref(ref) { }

		void run()
		{
			if (ref == trigger)
				state = true;
		}

		bool state;
		bool trigger;

	private:
		bool& ref;
	};
}

#endif /* __CH_NTB_EEDURO_LATCH_HPP_ */
