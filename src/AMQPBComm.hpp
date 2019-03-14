//
//  AMQPBComm.hpp
//  AMQPBComm
//
//  Created by Clyde Wu on 2017/9/14.
//  Copyright © 2017年 Clyde Wu. All rights reserved.
//

#ifndef AMQPBComm_hpp
#define AMQPBComm_hpp

#include <string>
#include <stdexcept>

#include <amqp_tcp_socket.h>
#include <amqp.h>
#include <amqp_framing.h>

#include <opencv2/core.hpp>

#include "hoa_frame.pb.h"

using namespace std;

namespace hoa
{
    static const string kDefaultVHost = "/";
    static const int kDefaultChannelMax = 0;
    static const int kDefaultChannel = 1;
    static const int kDefaultFrameMax = 5 * 1024 * 1024;
    static const int kDefaultHeartBeat = 0;
    static const string kDefaultUsername = "guest";
    static const string kDefaultPassword = "guest";
    static const amqp_sasl_method_enum kDefaultMethos = AMQP_SASL_METHOD_PLAIN;


    class AMQPBCommError: public runtime_error
    {
    public:
        AMQPBCommError(string what_arg): AMQPBCommError(what_arg, 0) {}
        AMQPBCommError(string what_arg, int err): runtime_error(what_arg), err_(err) {}
        int err_;
    };


    class AMQPBComm
    {
    public:
        AMQPBComm(const string& hostname, const int& port);
        AMQPBComm(const string& hostname, const int& port, const string& username, const string& password);
        
        void Connect();
        void Disconnect();
        void CreateQueue(const string& queuename);
        void DeleteQueue(const string& queuename);
        void DeleteQueue(const string& queuename, bool if_unused, bool if_empty);

        void Send(const string& queuename, hoa::HOAFrame& frame);
        
        void Recv(const string& queuename, hoa::HOAFrame& frame);
        
        ~AMQPBComm();
        
    private:
        string rmq_hostname_;
        int rmq_port_;
        string rmq_username_;
        string rmq_password_;
        int rmq_channel_;
        
        amqp_socket_t* p_rmq_socket_;
        amqp_connection_state_t rmq_conn_;
        
        inline void check_reply(amqp_rpc_reply_t& reply, const string& msg);
    };
}

#endif /* AMQPBComm_hpp */

