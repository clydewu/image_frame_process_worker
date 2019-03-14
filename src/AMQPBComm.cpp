//
//  AMQPBComm.cpp
//  AMQPBComm
//
//  Created by Clyde Wu on 2017/9/14.
//  Copyright © 2017年 Clyde Wu. All rights reserved.
//

#include<cstring>

#include "AMQPBComm.hpp"

using namespace std;
using namespace hoa;


AMQPBComm::AMQPBComm(const string& hostname, const int& port)
: AMQPBComm(hostname, port, kDefaultUsername, kDefaultPassword)
{
}


AMQPBComm::AMQPBComm(const string& hostname, const int& port, const string& username, const string& password)
: rmq_hostname_(hostname)
, rmq_port_(port)
, rmq_username_(username)
, rmq_password_(password)
, rmq_channel_(kDefaultChannel)
, p_rmq_socket_(nullptr)
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;
}


AMQPBComm::~AMQPBComm()
{
    google::protobuf::ShutdownProtobufLibrary();
    Disconnect();
}


void AMQPBComm::Connect()
{
    amqp_rpc_reply_t reply;
    
    rmq_conn_ = amqp_new_connection();
    if (rmq_conn_ == nullptr)
    {
        throw runtime_error("Create new connect failure");
    }
    
    p_rmq_socket_ = amqp_tcp_socket_new(rmq_conn_);
    if (p_rmq_socket_ == nullptr)
    {
        throw runtime_error("Create socket failure");
    }
    
    if (amqp_socket_open(p_rmq_socket_, rmq_hostname_.c_str(), rmq_port_) != 0)
    {
        throw runtime_error("Open socket failure");
    }
    
    reply = amqp_login(rmq_conn_, kDefaultVHost.c_str(), kDefaultChannelMax, kDefaultFrameMax, kDefaultHeartBeat, kDefaultMethos, rmq_username_.c_str(), rmq_password_.c_str());
    check_reply(reply, "RabbitMQ login failure");
    
    amqp_channel_open(rmq_conn_, rmq_channel_);
    reply = amqp_get_rpc_reply(rmq_conn_);
    check_reply(reply, "Open channel failure");
    
}


void AMQPBComm::Disconnect()
{
    //-- TODO, detect should we real need to disconnect
    amqp_channel_close(rmq_conn_, 1, AMQP_REPLY_SUCCESS);
    amqp_connection_close(rmq_conn_, AMQP_REPLY_SUCCESS);
    amqp_destroy_connection(rmq_conn_);
}


void AMQPBComm::CreateQueue(const string& queuename)
{
    amqp_bytes_t result_queue;
    amqp_rpc_reply_t reply;
    amqp_queue_declare_ok_t* result = amqp_queue_declare(rmq_conn_, kDefaultChannel, amqp_cstring_bytes(queuename.c_str()), 0, 0, 0, 1, amqp_empty_table);
    reply = amqp_get_rpc_reply(rmq_conn_);
    check_reply(reply, "Declare queue failure");
    
    result_queue = amqp_bytes_malloc_dup(result->queue);
    if (result_queue.bytes == nullptr)
    {
        throw AMQPBCommError("Declare queue failure, Can not get queue name back");
    }
}


void AMQPBComm::DeleteQueue(const string& queuename)
{
    amqp_rpc_reply_t reply;
    //    amqp_bytes_t result_queue;
    //    amqp_queue_delete_ok_t* result = amqp_queue_delete(rmq_conn_, kDefaultChannel, amqp_cstring_bytes(queuename.c_str()), true, true);
    amqp_queue_delete(rmq_conn_, kDefaultChannel, amqp_cstring_bytes(queuename.c_str()), true, true);
    reply = amqp_get_rpc_reply(rmq_conn_);
    check_reply(reply, "Delete queue failure");
}


void AMQPBComm::Send(const string& queuename, hoa::HOAFrame& frame)
{
    int status;
    amqp_bytes_t message;
    message.len = frame.ByteSize();
    message.bytes = malloc(message.len);
    frame.SerializeToArray(message.bytes, frame.ByteSize());
    
    if ((status = amqp_basic_publish(rmq_conn_, kDefaultChannel, amqp_empty_bytes , amqp_cstring_bytes(queuename.c_str()), 0, 0, NULL, message)) != AMQP_STATUS_OK) {
        free(message.bytes);
        throw AMQPBCommError("Send message failure", status);
    }
    free(message.bytes);
}


void AMQPBComm::Recv(const string& queuename, hoa::HOAFrame& frame)
{
    amqp_rpc_reply_t reply;
    amqp_message_t message;
    
    amqp_maybe_release_buffers(rmq_conn_);
    
    reply = amqp_basic_get(rmq_conn_, rmq_channel_, amqp_cstring_bytes(queuename.c_str()), true);
    check_reply(reply, "Basic get failure");
    
    reply = amqp_read_message(rmq_conn_, rmq_channel_, &message, 0);
    check_reply(reply, "Read message failure");
    
    frame.ParseFromArray(message.body.bytes, (int)message.body.len);
    amqp_destroy_message(&message);
}


inline void AMQPBComm::check_reply(amqp_rpc_reply_t& reply, const string& msg)
{
    if (reply.reply_type != AMQP_RESPONSE_NORMAL)
    {
        throw AMQPBCommError(msg, reply.reply_type);
    }
}

