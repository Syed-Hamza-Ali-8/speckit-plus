from utils.errors import error_response

class NotificationService:

    def send_email(self, to: str, subject: str, body: str):
        # Replace with real provider if spec requires
        print(f"[EMAIL] To: {to}\nSubject: {subject}\n\n{body}")

    def in_app(self, user_id: str, message: str, session):
        from backend.models.notification import Notification
        note = Notification(user_id=user_id, message=message)
        session.add(note)
        session.commit()

notification_service = NotificationService()
