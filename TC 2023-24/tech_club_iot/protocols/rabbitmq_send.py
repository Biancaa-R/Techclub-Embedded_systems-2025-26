# RabbitMQ messaging protocol:

import pika

connection=pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
channel=connection.channel()
# A broker on the local machine hence the local host
#In case of differnt machine specify ip

#Before sending the message the recipient queue should exist:
channel.queue_declare(queue="hello")
channel.basic_publish(exchange='',routing_key="hello",body="Hello world")
print("[x] sent hello world")
connection.close()