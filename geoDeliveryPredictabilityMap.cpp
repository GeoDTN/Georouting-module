/*
 * geoDeliveryPredictabilityMap.cpp
 *
 *  Created on: 08.01.2013
 *      Author: Tadewos Somano
 */

#include "routing/geoprophet/geoDeliveryPredictabilityMap.h"
#include "core/BundleCore.h"
#include <ibrdtn/utils/Clock.h>
#include <ibrcommon/Logger.h>
#include <vector>
#include<cmath>
namespace dtn
{
	namespace routing
	{
		const dtn::data::Number geoDeliveryPredictabilityMap::identifier = NodeHandshakeItem::GEODELIVERY_PREDICTABILITY_MAP;//geoDELIVERY_PREDICTABILITY_M;

		geoDeliveryPredictabilityMap::geoDeliveryPredictabilityMap()
		: NeighborDataSetImpl(geoDeliveryPredictabilityMap::identifier), _gamma(0.0), _beta(0.0),_lastAgingTime(0), _time_unit(0)
		{
		}

		geoDeliveryPredictabilityMap::geoDeliveryPredictabilityMap(const size_t &time_unit,const float &beta, const float &gamma)
		: NeighborDataSetImpl(geoDeliveryPredictabilityMap::identifier), _beta(beta), _gamma(gamma), _lastAgingTime(0), _time_unit(time_unit)/*,val14(),val15()*/
		{
		}

		geoDeliveryPredictabilityMap::~geoDeliveryPredictabilityMap() {
		}

		const dtn::data::Number& geoDeliveryPredictabilityMap::getIdentifier() const
		{
			return identifier;
		}

		dtn::data::Length geoDeliveryPredictabilityMap::getLength() const
		{

			dtn::data::Length len = 0;
			for(predictmap::const_iterator it = _predictmap.begin(); it != _predictmap.end(); ++it)
			{
				/* calculate length of the EID */
				const std::string eid = it->first.getString();
				dtn::data::Length eid_len = eid.length();
				len += data::Number(eid_len).getLength() + eid_len;

				/* calculate length of the float in fixed notation */
				const val17& values = it->second;
				std::stringstream ss;

ss<<values.val1<<values.val2<<values.val3<<values.val4<<values.val5<<values.val6<<values.val7<<std::flush;
				dtn::data::Length float_len = ss.str().length();
				len += data::Number(float_len).getLength() + float_len;
			}
			return data::Number(_predictmap.size()).getLength() + len;
		}

		std::ostream& geoDeliveryPredictabilityMap::serialize(std::ostream& stream) const
		{

			stream << data::Number(_predictmap.size());
              //stream << data::Values(_predictmap.size());
			for(predictmap::const_iterator it = _predictmap.begin(); it != _predictmap.end(); ++it)
			{
				const std::string eid = it->first.getString();
				stream << data::Number(eid.length()) << eid;

				const val17& values = it->second;

				std::stringstream ss;
				ss<<values.val1<<values.val2<<values.val3<<values.val4<<values.val5<<values.val6<<values.val7<<std::flush;
				stream << data::Number(ss.str().length());
				stream << ss.str();
			}
			IBRCOMMON_LOGGER_DEBUG_TAG("DeliveryPredictabilityMap", 20) << "Serialized with " << _predictmap.size() << " items." << IBRCOMMON_LOGGER_ENDL;
			IBRCOMMON_LOGGER_DEBUG_TAG("DeliveryPredictabilityMap", 60) << *this << IBRCOMMON_LOGGER_ENDL;
			return stream;
		}

		std::istream& geoDeliveryPredictabilityMap::deserialize(std::istream& stream)
		{
			val17 gps_local;
			data::Number elements_read(0);
			data::Number map_size;
			stream >> map_size;

			while(elements_read < map_size)
			{
				/* read the EID */
				data::Number eid_len;
				stream >> eid_len;

				// create a buffer for the EID
				std::vector<char> eid_cstr(eid_len.get<size_t>());

				// read the EID string
				stream.read(&eid_cstr[0], eid_cstr.size());

				// convert the string into an EID object
				dtn::data::EID eid(std::string(eid_cstr.begin(), eid_cstr.end()));

				if(eid == data::EID())
					throw dtn::InvalidDataException("EID could not be casted, while parsing a dp_map.");



				//gps_local=_gps_read.gps_values(values1);
				dtn::data::Number float_len;
				//dtn::data::Values float_len;
				stream >> float_len; //**********************************************************///

				// create a buffer for the data string
				std::vector<char> f_cstr(float_len.get<size_t>());

				// read the data string
				stream.read(&f_cstr[0], f_cstr.size());

				// convert string data into a stringstream
				std::stringstream ss(std::string(f_cstr.begin(), f_cstr.end()));

				// convert string data into a float
				//ss >> gps_local;
				//ss.read(reinterpret_cast<char*>(gps_local),sizeof(val)); //replacement by Tad
				ss>>gps_local.val1>>gps_local.val2>>gps_local.val3>>gps_local.val4>>gps_local.val5>>gps_local.val6>>gps_local.val7;//<<std::flush;

				if(ss.fail())
					throw dtn::InvalidDataException("Float could not be casted, while parsing a dp_map.");

				/* check if f is in a proper range */
				//if(f < 0 || f > 1)
if(!std::isnan(gps_local.val1) && !std::isnan(gps_local.val2) && !std::isnan(gps_local.val3) && !std::isnan(gps_local.val4)&& !std::isnan(gps_local.val5)&& !std::isnan(gps_local.val6)&& !std::isnan(gps_local.val7))
//printf("received:%f %f %f %f %f %f\n",values->val1,values->val2,values->val3,values->val4,values->val5,values->val6);
					continue;

				/* insert the data into the map */
				_predictmap[eid] = gps_local;

				elements_read += 1;
			}

			IBRCOMMON_LOGGER_DEBUG_TAG("DeliveryPredictabilityMap", 20) << "Deserialized with " << _predictmap.size() << " items." << IBRCOMMON_LOGGER_ENDL;
			IBRCOMMON_LOGGER_DEBUG_TAG("DeliveryPredictabilityMap", 60) << *this << IBRCOMMON_LOGGER_ENDL;
			return stream;
		}

		val17 geoDeliveryPredictabilityMap::get(const dtn::data::EID &neighbor) const throw (ValueNotFoundException)
		{
			predictmap::const_iterator it;
			if ((it = _predictmap.find(neighbor)) != _predictmap.end())
			{
				return it->second;
			}

			throw ValueNotFoundException();
		}

		void geoDeliveryPredictabilityMap::set(const dtn::data::EID &neighbor, val17 values)
		{
			_predictmap[neighbor] = values;//_set_value;
		}

		void geoDeliveryPredictabilityMap::clear()
		{
			_predictmap.clear();
		}

		size_t geoDeliveryPredictabilityMap::size() const
		{
			return _predictmap.size();
		}

		
void geoDeliveryPredictabilityMap::update(const dtn::data::EID &host_b, const geoDeliveryPredictabilityMap &dpm, const float &p_encounter_first)
		{
//////////////////////////////////////////////////////////////////////
//values=_gps_read.gps_values(values1);
//////////////////////////////////////////////////////////////////////
			float p_ab = 0.0f;
			val11 values1;
			val17 neighbor7,local7,values7;
                           float P_ac=0.0f;

			try {
                                //get(host_b)=1;
				values7=get(host_b);

				 val12 values2 ;
				 //val11 values1; ///********************************************//
values2.val1=values7.val1;
values2.val2=values7.val2;
values2.val3=values7.val3;
values2.val4=values7.val4;
values2.val5=values7.val5;
values2.val6=values7.val6;
values2.val7=values7.val7;
				 //values2 =get(host_b);

p_ab=1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),values2)+_gps_read.calculate_speed( _gps_read.gps_values(values1), values2)+1);
			} catch (const geoDeliveryPredictabilityMap::ValueNotFoundException&) {
				p_ab =0;//p_encounter_first;
			}

			/**
			 * Calculate transitive values
			 */
			for (predictmap::const_iterator it = dpm._predictmap.begin(); it != dpm._predictmap.end(); ++it)
			{
				const dtn::data::EID &host_c = it->first;
				 //coordinates of C reported by B
				neighbor7  = it->second;
				// do not update values for the origin host
				if (host_b.sameHost(host_c)) continue;


				// do not process values with our own EID
				if (dtn::core::BundleCore::local.sameHost(host_c)) continue;

				
predictmap::iterator dp_it;
				if ((dp_it = _predictmap.find(host_c)) != _predictmap.end()) {
					//val3 local=dp_it->second;//local coordinates entry about node C's
					val12 local,neighbor2;
					//local7=dp_it->second;


					local.val1=dp_it->second.val1;
					local.val2=dp_it->second.val2;
					local.val3=dp_it->second.val3;
					local.val4=dp_it->second.val4;
					local.val5=dp_it->second.val5;
					local.val6=dp_it->second.val6;
					local.val7=dp_it->second.val7;
					neighbor2.val1=neighbor7.val1;
					neighbor2.val2=neighbor7.val2;
					neighbor2.val3=neighbor7.val3;
					neighbor2.val4=neighbor7.val4;
					neighbor2.val5=neighbor7.val5;
					neighbor2.val6=neighbor7.val6;
					neighbor2.val7=neighbor7.val7;
//if (1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),local)+_gps_read.calculate_speed(_gps_read.gps_values(values1),local)+1)<_beta*1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),neighbor2)+_gps_read.calculate_speed(_gps_read.gps_values(values1),neighbor2)+1))
					if (1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),local)+_gps_read.calculate_speed(_gps_read.gps_values(values1),local)+1)<_beta*pow(_gamma,((values1.val7-neighbor2.val7)/_time_unit))*1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),neighbor2)+_gps_read.calculate_speed(_gps_read.gps_values(values1),neighbor2)+1))
					//if (1/(_gps_read.calculate_distance(values,neighbor)+_gps_read.calculate_speed(values,neighbor)+1)<1/(_gps_read.calculate_distance(local,values2)+_gps_read.calculate_speed(local,values2)+1))

					{dp_it->second =  neighbor7;}
                                       else 
                                       {
                                       dp_it->second  =dp_it->second;}//local;}//7;}//dp_it->second=local; ?

				} 
else
{
					_predictmap[host_c] =neighbor7;



				}
			}
		}

		void geoDeliveryPredictabilityMap::age(const float &p_first_threshold)
		{
			float dp=0.0;
			val11 values1;
			const dtn::data::Timestamp current_time = dtn::utils::Clock::getMonotonicTimestamp();

			// prevent double aging
			if (current_time <= _lastAgingTime) return;

			const dtn::data::Timestamp k =(current_time - _lastAgingTime) /_time_unit;

			predictmap::iterator it;
			for(it = _predictmap.begin(); it != _predictmap.end();)
			{
				if(it->first == dtn::core::BundleCore::local)
				{
					++it;
					continue;
				}
				val12 values2;
				values2.val1=it->second.val1;
				values2.val2=it->second.val2;
				values2.val3=it->second.val3;
				values2.val4=it->second.val4;
				values2.val5=it->second.val5;
				values2.val6=it->second.val6;
				values2.val7=it->second.val7;

				dp=1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),values2)+_gps_read.calculate_speed(_gps_read.gps_values(values1),values2)+1);
				dp*=pow(_gamma, k.get<int>());
				//1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),values2)+_gps_read.calculate_speed(_gps_read.gps_values(values1),values2)+1)*=pow(_gamma, k.get<int>());

					//if(1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),values2)+_gps_read.calculate_speed(_gps_read.gps_values(values1),values2)+1) < p_first_threshold)
				if(dp< p_first_threshold)
				{
					_predictmap.erase(it++);
				} else {
					++it;
				}
			}

			_lastAgingTime = current_time;
		}

		void geoDeliveryPredictabilityMap::toString(std::ostream &stream) const
		{
			predictmap::const_iterator it;
			for (it = _predictmap.begin(); it != _predictmap.end(); ++it)
			{
				//stream << it->first.getString() << ": " << it->second << std::endl;
				stream << it->first.getString() << ": " << it->second.val1<< it->second.val2<< it->second.val3<< it->second.val4<< it->second.val5<< it->second.val6<<std::endl;
			}
		}

		std::ostream& operator<<(std::ostream& stream, const geoDeliveryPredictabilityMap& map)
		{
			map.toString(stream);
			return stream;
		}

		void geoDeliveryPredictabilityMap::store(std::ostream &output) const
		{

			// get the current monotonic time-stamp difference
			const dtn::data::Timestamp monotonic_diff = dtn::utils::Clock::getTime() - dtn::utils::Clock::getMonotonicTimestamp();

			// get a absolute time-stamp
			const dtn::data::Timestamp absAgingTime = monotonic_diff + _lastAgingTime;

			// write last aged time-stamp
			output << absAgingTime;

			// store the number of map entries
			output << dtn::data::Number(_predictmap.size());

			for (predictmap::const_iterator it = _predictmap.begin(); it != _predictmap.end(); ++it)
			{
				const dtn::data::EID &peer = it->first;
				//val7& values7 = it->second;// what is &p_value?if it is probability of encounter, just delete it.
//Delta and p-encounter are no more needed.
				dtn::data::BundleString peer_entry(peer.getString());

				// write EID
				output << peer_entry;

				const val17& values = it->second;
				output<<values.val1<<values.val2<<values.val3<<values.val4<<values.val5<<values.val6<<values.val7<<std::flush;//.write(reinterpret_cast<char*>(it->second),sizeof(it->second));
			}
		}

		void geoDeliveryPredictabilityMap::restore(std::istream &input)
		{		_predictmap.clear();

			// get a absolute time-stamp
			dtn::data::Timestamp absAgingTime;

			// read last aged time-stamp
			input >> absAgingTime;

			// get the current monotonic time-stamp difference
			const dtn::data::Timestamp monotonic_diff = dtn::utils::Clock::getTime() - dtn::utils::Clock::getMonotonicTimestamp();
			const dtn::data::Timestamp monotonic_now = dtn::utils::Clock::getMonotonicTimestamp();

			// eliminate time-stamp which are in the future
			if (monotonic_now >= (absAgingTime - monotonic_diff))
			{
				// add entry to the map
				_lastAgingTime = absAgingTime - monotonic_diff;
			}
			else
			{
				// add entry to the map
				_lastAgingTime = monotonic_now;
			}

			dtn::data::Number num_entries;
			input >> num_entries;

			// silently fail
			while (input.good() && num_entries > 0)
			{
				dtn::data::BundleString peer_entry;
				val17 values7 ;
values7.val1=0;
values7.val2=0;
values7.val3=0;
values7.val4=0;
values7.val5=0;
values7.val6=0;
values7.val7=0;

				input >> peer_entry;
				//input.read(reinterpret_cast<char*>(values7), sizeof(val7));
input>>values7.val1>>values7.val2>>values7.val3>>values7.val4>>values7.val5>>values7.val6>>values7.val7;//>>values7.val1
				// add entry to the map
				_predictmap[dtn::data::EID(peer_entry)] = values7;

				num_entries--;
			}
		}
	} /* namespace routing */
} /* namespace dtn */
