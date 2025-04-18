# Rabbit mq reciever file:

import pika ,sys,os

def main():
    connection=pika.BlockingConnection(pika.ConnectionParameters(host="localhost"))
    channel=connection.channel()
    channel.queue_declare(queue="hello")

    #Recieving messages work by subscribing to a callback function

    def callback(ch,method,properties,body):
        print(f" [x] recieved {body}")
        #feels so similar to flask
        channel.basic_consume(queue="hello",auto_ack=True,on_message_callback=callback)
        print("[x] waiting for messages")
        channel.start_consuming()
if '__name__'=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Interupted")

        try:
            sys.exit(0)
        except:
            os._exit(0)

